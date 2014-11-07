/*
 *  Realmode interrupt call implementation.
 *  
 *
 *  Copyright (c) 2014 - CTU in Prague
 *                       Jan Doležal ( dolezj21@fel.cvut.cz )
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 *
 */

#include <bsp/int16.h>
#include <string.h>

#define IR_EAX_OFF      "0x00"
#define IR_EBX_OFF      "0x04"
#define IR_ECX_OFF      "0x08"
#define IR_EDX_OFF      "0x0C"
#define IR_ESI_OFF      "0x10"
#define IR_EDI_OFF      "0x14"
#define IR_DS_OFF       "0x18"
#define IR_ES_OFF       "0x1A"
#define IR_FS_OFF       "0x1C"
#define IR_GS_OFF       "0x1E"

#define BKP_ESP_OFF     "0x20"
#define BKP_SS_OFF      "0x24"
#define BKP_DS_OFF      "0x26"
#define RM_ENTRY        "0x28"
#define PM_ENTRY        "0x2C"

#define INT_REG_LEN     0x32
struct interrupt_registers_preserve_spots { /* used for passing parameters, fetching results and preserving values */
    struct interrupt_registers inoutregs;
    uint32_t pm_esp_bkp;
    uint16_t pm_ss_bkp;
    uint16_t ds_bkp;
    uint16_t rm_entry;
    uint16_t rm_code_segment;
    uint32_t pm_entry;
    uint16_t pm_code_selector;
    /* if modifying update INT_REG_LEN and offset definitions as well */
}__attribute__((__packed__));

#define BKP_IDTR_LIM    "0x00"
#define BKP_IDTR_BASE   "0x02"
#define BKP_ES_OFF      "0x06"
#define BKP_FS_OFF      "0x08"
#define BKP_GS_OFF      "0x0A"
#define RML_ENTRY       "0x0C"
#define RML_D_SEL       "0x12"
#define RM_SS           "0x14"
#define RM_ESP          "0x16"
#define RM_DS           "0x1A"
struct protected_mode_preserve_spots {
    uint16_t idtr_lim_bkp;
    uint32_t idtr_base_bkp;
    uint16_t es_bkp;
    uint16_t fs_bkp;
    uint16_t gs_bkp;
    uint32_t rml_entry;
    uint16_t rml_code_selector;
    uint16_t rml_data_selector;
    uint16_t rm_stack_segment;
    uint32_t rm_stack_pointer;
    uint16_t rm_data_segment;
}__attribute__((__packed__));

/* addresses where we are going to put Interrupt buffer, parameter/returned/preserved values, stack and copy code for calling BIOS interrupt real mode interface */
#define RM_INT_CALL_SPOT   0x1000
#define RM_INT_BUF_SPOT RM_INT_CALL_SPOT
#define RM_INT_BUF_LEN 512
#define INT_REGS_SPOT (RM_INT_BUF_SPOT+RM_INT_BUF_LEN)
/* position for real mode code reallocation to the first MB of RAM */
#define INT_FNC_SPOT (INT_REGS_SPOT+INT_REG_LEN)
#define INT_STACK_TOP (INT_FNC_SPOT+0x500)

/******************************
 * INT_BUF          * 512 B   *
 ******************************
 * INT_REGs         * 52 B    *
 ******************************
 * INT_FNC          *         *
 ******************** 0x500 B *
 * STACK            *         *
 ******************************/

#define __DP_TYPE 	uint8_t
#define __DP_YES 	((__DP_TYPE)1)
#define __DP_OK         __DP_YES
#define __DP_NO		((__DP_TYPE)-1)
#define __DP_FAIL	((__DP_TYPE)0)
static __DP_TYPE descsPrepared = __DP_NO;

/* rml - real mode alike */
#define rml_base  0x0
#define rml_limit 0xFFFF
static uint16_t rml_code_dsc_index = 0;
static uint16_t rml_data_dsc_index = 0;

static __DP_TYPE destroyRMDescriptors(void) {
    __DP_TYPE ret = __DP_OK;
    /* deallocate gdt entries */
    if(!i386_free_gdt_entry(rml_code_dsc_index))
    {
        /* selector to GDT out of range */
        ret = __DP_FAIL;
    }
    if(!i386_free_gdt_entry(rml_data_dsc_index))
    {
        /* selector to GDT out of range */
        ret = __DP_FAIL;
    }
    return ret;
}

/*
 * Prepares real-mode like descriptors to be used for switching
 * to real mode.
 *
 * @return __DP_YES descriptors are prepared
 * @return __DP_FAIL descriptors allocation failed (GDT too small)
 */
static __DP_TYPE prepareRMDescriptors (void) {
    /* check if descriptors were prepared already */
    if(descsPrepared == __DP_YES)
        return descsPrepared;
    /* create 'real mode like' segment descriptors, for switching to real mode */
    rml_code_dsc_index = i386_find_empty_gdt_entry();
    if(rml_code_dsc_index==0)
    {
        /* not enough space in GDT */
        descsPrepared = __DP_FAIL;
        return descsPrepared;
    }
    
    segment_descriptors flags_desc;
    memset(&flags_desc, 0, sizeof(flags_desc));
    flags_desc.type                = 0xE;      /* bits 4  */
    flags_desc.descriptor_type     = 0x1;      /* bits 1  */
    flags_desc.privilege           = 0x0;      /* bits 2  */
    flags_desc.present             = 0x1;      /* bits 1  */
    flags_desc.available           = 0x0;      /* bits 1  */
    flags_desc.fixed_value_bits    = 0x0;      /* bits 1  */
    flags_desc.operation_size      = 0x0;      /* bits 1  */
    flags_desc.granularity         = 0x0;      /* bits 1  */
    if(i386_put_gdt_entry(rml_code_dsc_index, rml_base, rml_limit, &flags_desc)==0)
    {
        /* selector to GDT out of range */
        destroyRMDescriptors();
        descsPrepared = __DP_FAIL;
        return descsPrepared;
    }

    rml_data_dsc_index = i386_find_empty_gdt_entry();
    if(rml_data_dsc_index==0)
    {
        /* not enough space in GDT for both descriptors */
        destroyRMDescriptors();
        descsPrepared = __DP_FAIL;
        return descsPrepared;
    }

    flags_desc.type                = 0x2;      /* bits 4  */
    if(i386_put_gdt_entry(rml_data_dsc_index, rml_base, rml_limit, &flags_desc)==0)
    {
        /* selector to GDT out of range */
        destroyRMDescriptors();
        descsPrepared = __DP_FAIL;
        return descsPrepared;
    }
    
    descsPrepared = __DP_YES;
    return descsPrepared;
}

inline void *i386_get_primary_rm_buffer() {
    return (void *)RM_INT_BUF_SPOT;
}

inline uint16_t i386_get_primary_rm_buffer_size() {
    return RM_INT_BUF_LEN;
}

int i386_real_interrupt_call(uint8_t interruptNumber, struct interrupt_registers *ir){
    uint32_t pagingon;
    volatile struct protected_mode_preserve_spots pm_bkp, *pm_bkp_addr;
    pm_bkp_addr = &pm_bkp;
    if(prepareRMDescriptors()!=__DP_YES)
	return 0;
    __asm__ volatile(   "\t"
        "movl    %%cr0, %%eax\n\t"
        "andl    %1, %%eax\n"
        : "=a"(pagingon)
        : "i"(CR0_PAGING)
    );
    if(pagingon)
        return 0;
    pm_bkp.rml_code_selector = (rml_code_dsc_index<<3);
    pm_bkp.rml_entry = INT_FNC_SPOT;
    pm_bkp.rml_data_selector = (rml_data_dsc_index<<3);
    pm_bkp.rm_stack_segment = 0;
    pm_bkp.rm_stack_pointer = INT_STACK_TOP;
    pm_bkp.rm_data_segment = 0;
    struct interrupt_registers_preserve_spots *parret = (struct interrupt_registers_preserve_spots *)INT_REGS_SPOT;
    parret->inoutregs = *ir;
    /* offset from the beginning of coppied code */
    uint16_t rm_entry_offset;
    __asm__ volatile(
        "movw   $(rment-cp_beg), %0\n\t"
        : "=r"(rm_entry_offset)
    );
    parret->rm_entry = INT_FNC_SPOT+rm_entry_offset;
    parret->rm_code_segment = 0;
    uint32_t pm_entry_offset;
    uint16_t current_code_selector;
    __asm__ volatile(
        "movl   $(cp_end), %0\n\t"
        "movw   %%cs, %1\n\t"
        : "=r"(pm_entry_offset), "=r"(current_code_selector)
    );
    parret->pm_entry = pm_entry_offset;
    parret->pm_code_selector = current_code_selector;
        /* copy desired code to first 64kB of RAM */
    __asm__ volatile(   "\t"
        "movl    $intins, %%ecx\n\t"
        "movb    %[int_no], 0x1(%%ecx)\n\t" /* write interrupt number */
        /* copy code for switching to real mode and executing interrupt to low memory */
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %[fnc_spot], %%edi\n\t"
        "rep movsb\n\t"
        /* backup stack */
        "movl    %[regs_spot], %%ebx\n\t"
        "movl    %%esp, "BKP_ESP_OFF"(%%ebx)\n\t"
        "movw    %%ss,  "BKP_SS_OFF"(%%ebx)\n\t"
        "movw    %%ds,  "BKP_DS_OFF"(%%ebx)\n\t"
        /* backup selectors */
        "movl    %[pm_bkp], %%esi\n\t"
        "movw    %%es, "BKP_ES_OFF"(%%esi)\n\t"
        "movw    %%fs, "BKP_FS_OFF"(%%esi)\n\t"
        "movw    %%gs, "BKP_GS_OFF"(%%esi)\n\t"
        /* hopefully loader does not damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */
        /* preserve idtr */
        "movl    %%esi, %%eax\n\t"
        "addl    $"BKP_IDTR_LIM", %%eax\n\t"
        "cli\n\t"
        "sidt    (%%eax)\n\t"
        "movl    $rmidt, %%eax\n\t"
        "lidt    (%%eax)\n\t"
        /* jump to copied function and */
        /* prepare 'real mode like' data selector */
        "movw    "RML_D_SEL"(%%esi), %%ax\n\t"
        /* prepare real mode data segment value */
        "xorl    %%edx,%%edx\n\t"
        "movw    "RM_DS"(%%esi), %%dx\n\t"
        /* prepare real mode stack values */
        "movw    "RM_SS"(%%esi), %%cx\n\t"
        "movl    "RM_ESP"(%%esi), %%esp\n\t"
        /* load 'real mode like' code selector */
        "ljmp   *"RML_ENTRY"(%%esi)\n"
"rmidt:"/* limit and base for realmode interrupt descriptor table */
        ".word 0x3FF\n\t"
        ".long 0\n\t"
        /* load 'real mode like' data selectors */
"cp_beg: .code16\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"
        /* disable protected mode */
        "movl    %%cr0, %%eax\n\t"
        "andl    %[cr0_prot_dis], %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* flush prefetch queue by far jumping */
        /* change selector in segmentation register to establish real mode style segment */
        "ljmp    *"RM_ENTRY"(%%ebx)\n\t"
"rment: "
        /* establish rm stack - esp was already set in 32-bit protected mode*/
        "movw    %%cx, %%ss\n\t"
        /* set data segment (value prepared in 32-bit prot mode) */
        "movw    %%dx, %%ds\n\t"
        /* count real mode pointer so we don't need to overuse address prefix (by using 32bit address) */
        "shll    $4,%%edx\n\t"
        "subl    %%edx,%%ebx\n\t"
        /* prepare values to be used after interrupt call */
        "pushw   %%bx\n\t"
        "pushw   %%ds\n\t"
        /* fill registers with parameters */
        "movw    " IR_DS_OFF"(%%bx), %%ax\n\t"
        "pushw   %%ax\n\t"
        "movl    "IR_EAX_OFF"(%%bx), %%eax\n\t"
        "movl    "IR_ECX_OFF"(%%bx), %%ecx\n\t"
        "movl    "IR_EDX_OFF"(%%bx), %%edx\n\t"
        "movl    "IR_EDI_OFF"(%%bx), %%edi\n\t"
        "movl    "IR_ESI_OFF"(%%bx), %%esi\n\t"
        "movw    " IR_ES_OFF"(%%bx), %%es\n\t"
        "movw    " IR_FS_OFF"(%%bx), %%fs\n\t"
        "movw    " IR_GS_OFF"(%%bx), %%gs\n\t"
        /* prepare ebx register */
        "movl    "IR_EBX_OFF"(%%bx), %%ebx\n\t"
        /* prepare ds */
        "popw    %%ds\n\t"
"intins: int     $0x0\n\t"
        /* fill return structure */
        "pushw   %%ds\n\t"
        "pushl   %%ebx\n\t"
        "movw    0x6(%%esp), %%ds\n\t"
        "movw    0x8(%%esp),%%bx\n\t" /* regs_spot */
        "movl    %%eax,"IR_EAX_OFF"(%%bx)\n\t"
        "popl    %%eax\n\t"
        "movl    %%eax,"IR_EBX_OFF"(%%bx)\n\t"
        "movl    %%ecx,"IR_ECX_OFF"(%%bx)\n\t"
        "movl    %%edx,"IR_EDX_OFF"(%%bx)\n\t"
        "movl    %%esi,"IR_ESI_OFF"(%%bx)\n\t"
        "movl    %%edi,"IR_EDI_OFF"(%%bx)\n\t"
        "popw    %%ax\n\t"
        "movw    %%ax, " IR_DS_OFF"(%%bx)\n\t"
        "movw    %%es, " IR_ES_OFF"(%%bx)\n\t"
        "movw    %%fs, " IR_FS_OFF"(%%bx)\n\t"
        "movw    %%gs, " IR_GS_OFF"(%%bx)\n\t"
        /* prepare protected mode data segment */
        "movw    "BKP_DS_OFF"(%%bx), %%ax\n\t"
        /* restore protected mode stack values */
        "movl    "BKP_ESP_OFF"(%%bx),%%esp\n\t"
        "movw    "BKP_SS_OFF"(%%bx), %%dx\n\t"
        /* return to protected mode */
        "movl    %%cr0, %%ecx     \n\t"
        "or      %[cr0_prot_ena], %%cx\n\t"
        "movl    %%ecx, %%cr0     \n\t"
        "ljmpl   *"PM_ENTRY"(%%bx)\n\t"
        ".code32\n"
        /* reload segmentation registers */
"cp_end:"
        "movw    %%ax, %%ds\n\t"
        /* restore stack segment in protected mode context */
        "movw    %%dx, %%ss\n\t"
        "movl    %[pm_bkp], %%esi\n\t"
        "movw    "BKP_ES_OFF"(%%esi), %%es\n\t"
        "movw    "BKP_FS_OFF"(%%esi), %%fs\n\t"
        "movw    "BKP_GS_OFF"(%%esi), %%gs\n\t"
        /* restore IDTR */
        "addl    $"BKP_IDTR_LIM", %%esi\n\t"
        "lidt    (%%esi)\n\t"
        : 
        : [fnc_spot]"i"(INT_FNC_SPOT), [regs_spot]"i"(INT_REGS_SPOT), [pm_bkp]"m"(pm_bkp_addr), [cr0_prot_ena]"i"(CR0_PROTECTION_ENABLE), [cr0_prot_dis]"i"(~CR0_PROTECTION_ENABLE), [int_no]"a"(interruptNumber)
        : "memory", "ebx", "ecx", "edx", "esi", "edi"
    );
    *ir = parret->inoutregs;
    return 1;
}


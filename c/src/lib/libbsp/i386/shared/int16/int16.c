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
#define BKP_IDTR_LIM    "0x24"
#define BKP_IDTR_BASE   "0x26"
#define BKP_DS_OFF      "0x2A"
#define BKP_ES_OFF      "0x2C"
#define BKP_FS_OFF      "0x2E"
#define BKP_GS_OFF      "0x30"
#define BKP_SS_OFF      "0x32"

#define INT_REG_LEN     0x34
struct interrupt_registers_preserve_spots { /* used for passing parameters, fetching results and preserving values */
    struct interrupt_registers inoutregs;
    uint32_t reg_esp_bkp;
    uint16_t idtr_lim_bkp;
    uint32_t idtr_base_bkp;
    uint16_t ds_bkp;
    uint16_t es_bkp;
    uint16_t fs_bkp;
    uint16_t gs_bkp;
    uint16_t ss_bkp;
    /* if modifying update INT_REG_LEN and offset definitions as well */
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
    uint16_t rml_code_dsc_selector = (rml_code_dsc_index<<3);
    uint16_t rml_data_dsc_selector = (rml_data_dsc_index<<3);
    struct interrupt_registers_preserve_spots *parret = (struct interrupt_registers_preserve_spots *)INT_REGS_SPOT;
    parret->inoutregs = *ir;
        /* copy desired code to first 64kB of RAM */
    __asm__ volatile(   "\t"
        "movl    $intins, %%ecx\n\t"
        "movb    %[int_no], 0x1(%%ecx)\n\t" /* write interrupt number */
        "movl    $rmlcsel, %%ecx\n\t"
        "movw    %[rml_code_sel], %%ax\n\t"
        "movw    %%ax, 0x5(%%ecx)\n\t" /* write real mode like code selector */
        /* backup current selectors */
        "movw    %%cs, %%ax\n\t"
        "movl    $curcs, %%ecx\n\t"
        "movw    %%ax, 0x6(%%ecx)\n\t"
        "movl    %[regs_spot], %%esi\n\t"
        "movw    %%ds, "BKP_DS_OFF"(%%esi)\n\t"
        "movw    %%es, "BKP_ES_OFF"(%%esi)\n\t"
        "movw    %%fs, "BKP_FS_OFF"(%%esi)\n\t"
        "movw    %%gs, "BKP_GS_OFF"(%%esi)\n\t"
        "movw    %%ss, "BKP_SS_OFF"(%%esi)\n\t"
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %[fnc_spot], %%edi\n\t"
        "rep movsb\n\t"
        "cli\n\t"
        /* hopefully loader does not damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */
        /* preserve idtr */
        "movl    %[regs_spot]+"BKP_IDTR_LIM", %%eax\n\t"
        "sidt    (%%eax)\n\t"
        "movl    $rmidt, %%eax\n\t"
        "lidt    (%%eax)\n\t"
        /* jump to copied function */
        "movl    %[fnc_spot], %%eax\n\t"
        "jmp     *%%eax\n"
        /* limit and base for realmode interrupt descriptor table */
"rmidt:"
        ".word 0x3FF\n\t"
        ".long 0\n\t"
        /* load 'real mode like' selectors */
"cp_beg: movw    %[rml_data_sel], %%ax\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"

        /* load 'real mode like' code selector */
"rmlcsel:ljmp    $0x00, %[fnc_spot]+(cs_real-cp_beg)\n"
"cs_real:"
        ".code16\n\t"
        /* disable protected mode */
        "movl    %%cr0, %%eax\n\t"
        "andl    %[cr0_prot_dis], %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* flush prefetch queue by far jumping */
        /* change selector in segmentation register to correct real mode style segment */
        "ljmp    $0x0, %[fnc_spot]+(dsels-cp_beg)\n"
"dsels:  xor     %%ax, %%ax\n\t"
        "mov     %%ax, %%ss\n\t"
        "mov     %%ax, %%ds\n\t"
        "mov     %%ax, %%fs\n\t"
        "mov     %%ax, %%gs\n\t"
        "movl    %[regs_spot], %%esi\n\t"
        /* backup stack pointer */
        "movl    %%esp, "BKP_ESP_OFF"(%%esi)\n\t"
        /* establish rm stack */
        "movl    %[stack_top], %%esp\n\t"
        /* fill registers with parameters */
        "movl    "IR_EAX_OFF"(%%esi), %%eax\n\t"
        "movl    "IR_EBX_OFF"(%%esi), %%ebx\n\t"
        "movl    "IR_ECX_OFF"(%%esi), %%ecx\n\t"
        "movl    "IR_EDX_OFF"(%%esi), %%edx\n\t"
        "movl    "IR_EDI_OFF"(%%esi), %%edi\n\t"
        "movw    " IR_DS_OFF"(%%esi), %%ds\n\t"
        "movw    " IR_ES_OFF"(%%esi), %%es\n\t"
        "movw    " IR_FS_OFF"(%%esi), %%fs\n\t"
        "movw    " IR_GS_OFF"(%%esi), %%gs\n\t"
        /* prepare esi register */
        "movl    "IR_ESI_OFF"(%%esi), %%esi\n\t"
"intins: int     $0x0\n\t"

        /* fill return structure */
        "pushl   %%esi\n\t"
        "movl    %[regs_spot], %%esi\n\t"
        "movl    %%eax,"IR_EAX_OFF"(%%esi)\n\t"
        "popl    %%eax\n\t"
        "movl    %%eax,"IR_ESI_OFF"(%%esi)\n\t"
        "movl    %%ebx,"IR_EBX_OFF"(%%esi)\n\t"
        "movl    %%ecx,"IR_ECX_OFF"(%%esi)\n\t"
        "movl    %%edx,"IR_EDX_OFF"(%%esi)\n\t"
        "movl    %%edi,"IR_EDI_OFF"(%%esi)\n\t"
        "movw    %%ds, " IR_DS_OFF"(%%esi)\n\t"
        "movw    %%es, " IR_ES_OFF"(%%esi)\n\t"
        "movw    %%fs, " IR_FS_OFF"(%%esi)\n\t"
        "movw    %%gs, " IR_GS_OFF"(%%esi)\n\t"
        /* restore original stack pointer */
        "movl    "BKP_ESP_OFF"(%%esi),%%esp\n\t"
"aftint:"
        /* return to protected mode */
        "movl    %%cr0, %%eax     \n\t"
        "orl     %[cr0_prot_ena], %%eax    \n\t"
        "movl    %%eax, %%cr0     \n\t"
"curcs:  ljmpl   $0x0,$cp_end \n\t"
        ".code32\n"
        /* reload segmentation registers */
"cp_end: movl    %[regs_spot], %%esi\n\t"
        "movw    "BKP_DS_OFF"(%%esi), %%ds\n\t"
        "movw    "BKP_ES_OFF"(%%esi), %%es\n\t"
        "movw    "BKP_FS_OFF"(%%esi), %%fs\n\t"
        "movw    "BKP_GS_OFF"(%%esi), %%gs\n\t"
        "movw    "BKP_SS_OFF"(%%esi), %%ss\n\t"
        /* restore IDTR */
        "movl    %[regs_spot]+"BKP_IDTR_LIM", %%eax\n\t"
        "lidt    (%%eax)\n\t"
        : 
        : [fnc_spot]"i"(INT_FNC_SPOT), [regs_spot]"i"(INT_REGS_SPOT), [stack_top]"i"(INT_STACK_TOP), [cr0_prot_ena]"i"(CR0_PROTECTION_ENABLE), [cr0_prot_dis]"i"(~CR0_PROTECTION_ENABLE), [int_no]"a"(interruptNumber), [rml_code_sel]"m"(rml_code_dsc_selector), [rml_data_sel]"m"(rml_data_dsc_selector)
        : "memory", "ebx", "ecx", "edx", "esi", "edi"
    );
    *ir = parret->inoutregs;
    return 1;
}


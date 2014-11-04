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

#define INT_REG_LEN 0x2A
struct interrupt_registers_preserve_spots { /* used for passing parameters, fetching results and preserving values */
    struct interrupt_registers inoutregs;       /* off 0x00 */
    uint32_t reg_esp_bkp;                       /* off 0x20 */
    uint16_t idtr_lim_bkp;                      /* off 0x24 */
    uint32_t idtr_base_bkp;                     /* off 0x26 */
    /* if adding new element update INT_REG_LEN as well */
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
 * INT_REGs         * 36 B    *
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

inline void *get_primary_rm_buffer() {
    return (void *)RM_INT_BUF_SPOT;
}

inline uint16_t get_primary_rm_buffer_size() {
    return RM_INT_BUF_LEN;
}

int BIOSinterruptcall(uint8_t interruptNumber, struct interrupt_registers *ir){
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
        "movb    %5, 0x1(%%ecx)\n\t" /* write interrupt number */
        "movl    $rmlcsel, %%ecx\n\t"
        "movw    %6, %%ax\n\t"
        "movw    %%ax, 0x5(%%ecx)\n\t" /* write real mode like code selector */
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %0, %%edi\n\t"
        "rep movsb\n\t"
        "cli\n\t"
        /* jump to copied function */
        "movl    %0, %%eax\n\t"
        "jmp     *%%eax\n"
        /* load 'real mode like' selectors */
"cp_beg: movw    %7, %%ax\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"

        /* load 'real mode like' code selector */
"rmlcsel:ljmp    $0x00, %0+(cs_real-cp_beg)\n"
"cs_real:"
        ".code16\n\t"
        /* disable protected mode */
        "movl    %%cr0, %%eax\n\t"
        "andl    %4, %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* hopefully loader does not damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */
        /* preserve idtr */
        "movl    %1+0x24, %%eax\n\t"
        "sidt    (%%eax)\n\t"
        "movl    %0+(begidt-cp_beg), %%eax\n\t"
        "lidt    (%%eax)\n\t"
        /* flush prefetch queue by far jumping */
        /* change selector in segmentation register to correct real mode style segment */
        "ljmp    $0x0, %0+(dsels-cp_beg)\n"
        /* limit and base for realmode interrupt descriptor table */
"begidt:"
        ".word 0x3FF\n\t"
        ".long 0\n\t"
"dsels:  xor     %%ax, %%ax\n\t"
        "mov     %%ax, %%ss\n\t"
        "mov     %%ax, %%ds\n\t"
        "mov     %%ax, %%fs\n\t"
        "mov     %%ax, %%gs\n\t"
        /* fill registers with parameters */
        "movl    %1, %%esi\n\t"
        "movl    0x00(%%esi), %%eax\n\t"
        "movl    0x04(%%esi), %%ebx\n\t"
        "movl    0x08(%%esi), %%ecx\n\t"
        "movl    0x0C(%%esi), %%edx\n\t"
        "movl    0x14(%%esi), %%edi\n\t"
        "movw    0x18(%%esi), %%ds\n\t"
        "movw    0x1A(%%esi), %%es\n\t"
        "movw    0x1C(%%esi), %%fs\n\t"
        "movw    0x1E(%%esi), %%gs\n\t"
        /* backup stack pointer */
        "movl    %%esp, 0x20(%%esi)\n\t"
        /* prepare esi register */
        "movl    0x10(%%esi), %%esi\n\t"
        /* establish rm stack */
        "movl    %2, %%esp\n\t"
"intins: int     $0x0\n\t"

        /* fill return structure */
        "pushl   %%esi\n\t"
        "movl    %1, %%esi\n\t"
        "movl    %%eax,0x00(%%esi)\n\t"
        "popl    %%eax\n\t"
        "movl    %%eax,0x10(%%esi)\n\t" /* store returned esi */
        "movl    %%ebx,0x04(%%esi)\n\t"
        "movl    %%ecx,0x08(%%esi)\n\t"
        "movl    %%edx,0x0C(%%esi)\n\t"
        "movl    %%edi,0x14(%%esi)\n\t"
        "movw    %%ds, 0x18(%%esi)\n\t"
        "movw    %%es, 0x1A(%%esi)\n\t"
        "movw    %%fs, 0x1C(%%esi)\n\t"
        "movw    %%gs, 0x1E(%%esi)\n\t"
        /* restore original stack pointer */
        "movl    0x20(%%esi),%%esp\n\t"
"aftint:"
        /* return to protected mode */
        "movl    %%cr0, %%eax     \n\t"
        "orl     %3, %%eax    \n\t"
        "movl    %%eax, %%cr0     \n\t"
        "ljmpl   $0x8,$cp_end \n\t"
        ".code32\n"
        /* reload segmentation registers */
"cp_end: movw    $0x10,%%ax\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"
        /* restore IDTR */
        "movl    %1+0x24, %%eax\n\t"
        "lidt    (%%eax)\n\t"
        : 
        : "i"(INT_FNC_SPOT), "i"(INT_REGS_SPOT), "i"(INT_STACK_TOP), "i"(CR0_PROTECTION_ENABLE), "i"(~CR0_PROTECTION_ENABLE), "a"(interruptNumber), "m"(rml_code_dsc_selector), "m"(rml_data_dsc_selector)
        : "memory", "ebx", "ecx", "edx", "esi", "edi"
    );
    *ir = parret->inoutregs;
    return 1;
}


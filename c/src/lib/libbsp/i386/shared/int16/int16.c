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


inline void *get_primary_rm_buffer() {
    return (void *)RM_INT_BUF_SPOT;
}

inline uint16_t get_primary_rm_buffer_size() {
    return RM_INT_BUF_LEN;
}

/**
 * This function presumes prepared real mode like descriptors for code (index 4 - selector 0x20) and data (index 3 - selector 0x18) in the GDT.
 */
void BIOSinterruptcall(uint8_t interruptNumber, struct interrupt_registers *ir){
    struct interrupt_registers *parret = (struct interrupt_registers *)INT_REGS_SPOT;
    *parret = *ir;
        /* copy desired code to first 64kB of RAM */
    __asm__ volatile(   "\t"
        "movl    $intins, %%ebx\n\t"
        "movb    %6, 0x1(%%ebx)\n\t" /* write interrupt number */
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %0, %%edi\n\t"
        "rep movsb\n\t"
        /* test that paging is off */
        "movl    %%cr0, %%eax\n\t"
        "testl   %3, %%eax\n"
"pg_dis: jne     pg_dis\n\t" /* hopefuly no loader turns on paging */
        "cli\n\t"
        /* jump to copied function */
        "movl    %0, %%eax\n\t"
        "jmp     *%%eax\n"
        /* load 'real mode like' selectors */
"cp_beg: movw    $0x20, %%ax\n\t" /* fourth element of GDT */
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"

        /* load 'real mode like' code selector */
        "ljmp    $0x18, %0+(cs_real-cp_beg)\n"
"cs_real:"
        ".code16\n\t"
        /* disable protected mode */
        "movl    %%cr0, %%eax\n\t"
        "andl    %5, %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* hopefully loader does not damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */
        /* preserve idtr */
        "movl    %1+0x1E, %%eax\n\t"
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
        "movw    0x18(%%esi), %%es\n\t"
        /* backup stack pointer */
        "movl    %%esp, 0x1A(%%esi)\n\t"
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
        "movw    %%es, 0x18(%%esi)\n\t"
        /* restore original stack pointer */
        "movl    0x1A(%%esi),%%esp\n\t"
"aftint:"
        /* return to protected mode */
        "movl    %%cr0, %%eax     \n\t"
        "orl     %4, %%eax    \n\t"
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
        "movl    %1+0x1E, %%eax\n\t"
        "lidt    (%%eax)\n\t"
        : 
        : "i"(INT_FNC_SPOT), "i"(INT_REGS_SPOT), "i"(INT_STACK_TOP), "i"(CR0_PAGING), "i"(CR0_PROTECTION_ENABLE), "i"(~CR0_PROTECTION_ENABLE), "a"(interruptNumber)
        : "memory", "ebx", "ecx", "edx", "esi", "edi"
    );
}


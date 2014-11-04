/**
 * @file int16.h
 *
 * @ingroup i386_shared
 *
 * @brief Definitioins supporting real mode interrupt calls.
 */

/*
 *
 *
 * Copyright (C) 2014  Jan Doležal (dolezj21@fel.cvut.cz)
 *                     CTU in Prague.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <libcpu/cpu.h>
#include <stdint.h>

/* --- BIOS service interrupt number --- */
/* number of interrupt servicing video functions */
#define INTERRUPT_NO_VIDEO_SERVICES 0x10

struct interrupt_registers { /* used for passing parameters, fetching results and preserving values */
    uint32_t reg_eax;                           /* off 0x0  */
    uint32_t reg_ebx;                           /* off 0x4  */
    uint32_t reg_ecx;                           /* off 0x8  */
    uint32_t reg_edx;                           /* off 0xC  */
    uint32_t reg_esi;                           /* off 0x10 */
    uint32_t reg_edi;                           /* off 0x14 */
    uint16_t reg_es;                            /* off 0x18 */
}__attribute__((__packed__));

/**
 * provides position to real mode buffer, which is buffer
 * accessible from real mode context - it is located below
 * address ~0x100000 in order for it to be accessible
 * This buffer is meant to be pointed to by segReg:GenPurpReg
 * and through this get bigger portion of information to/from
 * interrupt service routine.
 *
 * @return pointer to buffer
 */
extern void *get_primary_rm_buffer(void);

/**
 * returns size of real mode buffer
 *
 * @return size of buffer
 */
extern uint16_t get_primary_rm_buffer_size(void);

/* if there is ever need to have more buffers or setting size of the buffer,
 * feel free to modify the code */

/**
 * This function allows calling interrupts in real mode and to set processor
 * registers as desired before interrupt call is made and to retrieve the
 * registers content after call was made.
 * 
 * @param interruptNumber interrupt number to be called
 * @param ir pointer to structure containing registers to be passed to interrupt
 *        and to retrieve register content after call was made.
 * @return  0 call failed
 *          1 call successful
 */
extern int BIOSinterruptcall(uint8_t interruptNumber, struct interrupt_registers *ir);


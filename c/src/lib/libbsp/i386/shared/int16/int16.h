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

#define INT_REG_LEN 0x24
struct interrupt_registers { /* used for passing parameters, fetching results and preserving values */
    uint32_t reg_eax;                           /* off 0x0  */
    uint32_t reg_ebx;                           /* off 0x4  */
    uint32_t reg_ecx;                           /* off 0x8  */
    uint32_t reg_edx;                           /* off 0xC  */
    uint32_t reg_esi;                           /* off 0x10 */
    uint32_t reg_edi;                           /* off 0x14 */
    uint16_t reg_es;                            /* off 0x18 */
    uint32_t reg_esp_bkp;                       /* off 0x1A */
    uint16_t idtr_lim_bkp;                      /* off 0x1E */
    uint32_t idtr_base_bkp;                     /* off 0x20 */
    /* if adding new element update INT_REG_LEN as well */
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
void *get_primary_rm_buffer();

/**
 * returns size of real mode buffer
 *
 * @return size of buffer
 */
uint16_t get_primary_rm_buffer_size();

/* if there is ever need to have more buffers or setting size of the buffer,
 * feel free to modify the code */

/**
 * 
 * 
 * @param interrupt number to be called
 */
void BIOSinterruptcall(uint8_t interruptNumber, struct interrupt_registers *ir);


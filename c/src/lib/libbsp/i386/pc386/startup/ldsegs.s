/*-------------------------------------------------------------------------+
| ldsegs.s v1.1 - PC386 BSP - 1997/08/07
+--------------------------------------------------------------------------+
| This file assists the board independent startup code by loading the proper
| segment register values. The values loaded are board dependent. In addition
| it contains code to enable the A20 line and to reprogram the PIC to relocate
| the IRQ interrupt vectors to 0x20 -> 0x2f.
| NOTE: No stack has been established when this routine is invoked.
|	It returns by jumping back to bspentry.
+--------------------------------------------------------------------------+
| (C) Copyright 1997 -
| - NavIST Group - Real-Time Distributed Systems and Industrial Automation
|
| http://pandora.ist.utl.pt
|
| Instituto Superior Tecnico * Lisboa * PORTUGAL
+--------------------------------------------------------------------------+
| Disclaimer:
|
| This file is provided "AS IS" without warranty of any kind, either
| expressed or implied.
+--------------------------------------------------------------------------+
| This code is base on:
|   ldsegs.s,v 1.4 1996/04/20 16:48:30 joel Exp - go32 BSP
| With the following copyright notice:
| **************************************************************************
| *  COPYRIGHT (c) 1989-1998.
| *  On-Line Applications Research Corporation (OAR).
| *  Copyright assigned to U.S. Government, 1994. 
| *
| *  The license and distribution terms for this file may be
| *  found in found in the file LICENSE in this distribution or at
| *  http://www.OARcorp.com/rtems/license.html.
| **************************************************************************
|
|  $Id$
|
| Also based on (from the Linux source tree):
|   setup.S - Copyright (C) 1991, 1992 Linus Torvalds
+--------------------------------------------------------------------------*/


#include "asm.h"

/*----------------------------------------------------------------------------+
| CODE section
+----------------------------------------------------------------------------*/
EXTERN (rtems_i8259_masks)
	
BEGIN_CODE

        EXTERN (_establish_stack)
	EXTERN (Timer_exit)
	EXTERN (clockOff)

	.p2align 4
/*----------------------------------------------------------------------------+
| delay
+------------------------------------------------------------------------------
| Delay is needed after doing I/O. We do it by writing to a non-existent port.
+----------------------------------------------------------------------------*/
SYM(delay):
	outb	al, $0xED	# about 1uS delay
	ret

/*-------------------------------------------------------------------------+
|         Function: _load_segments
|      Description: Current environment is standard PC booted by grub.
|                   So, there is no value in saving current GDT and IDT
|                   settings we have to set it up ourseves. (Naturally
|	            it will be not so in case we are booted by some
|                   boot monitor, however, then it will be different
|                   BSP). After that we have to load board segment registers
|                   with apropriate values +  reprogram PIC.
| Global Variables: None.
|        Arguments: None.
|          Returns: Nothing. 
+--------------------------------------------------------------------------*/
	.p2align 4
	
        PUBLIC (_load_segments)
SYM (_load_segments):

	lgdt SYM(gdtdesc)
	lidt SYM(idtdesc)

	/* Load CS, flush prefetched queue */
	ljmp $0x8, $next_step

next_step:	
        /* Load segment registers */
	movw $0x10, ax
	movw ax, ss
	movw ax, ds
	movw ax, es
	movw ax, fs
	movw ax, gs

/*---------------------------------------------------------------------+
| Now we have to reprogram the interrupts :-(. We put them right after
| the intel-reserved hardware interrupts, at int 0x20-0x2F. There they
| won't mess up anything. Sadly IBM really messed this up with the
| original PC, and they haven't been able to rectify it afterwards. Thus
| the bios puts interrupts at 0x08-0x0f, which is used for the internal
| hardware interrupts as well. We just have to reprogram the 8259's, and
| it isn't fun.
+---------------------------------------------------------------------*/

	movb	$0x11, al		/* initialization sequence          */
	outb	al, $0x20		/* send it to 8259A-1               */
	call	SYM(delay)
	outb	al, $0xA0		/* and to 8259A-2                   */
	call	SYM(delay)
	
	movb	$0x20, al		/* start of hardware int's (0x20)   */
	outb	al, $0x21
	call	SYM(delay)
	movb	$0x28, al		/* start of hardware int's 2 (0x28) */
	outb	al, $0xA1
	call	SYM(delay)
	
	movb	$0x04, al		/* 8259-1 is master                 */
	outb	al, $0x21
	call	SYM(delay)
	movb	$0x02, al		/* 8259-2 is slave                  */
	outb	al, $0xA1
	call	SYM(delay)
	
	movb	$0x01, al		/* 8086 mode for both               */
	outb	al, $0x21
	call	SYM(delay)
	outb	al, $0xA1
	call	SYM(delay)
	
	movb	$0xFF, al		/* mask off all interrupts for now  */
	outb	al, $0xA1
	call	SYM(delay)
	movb	$0xFB, al		/* mask all irq's but irq2 which    */
	outb	al, $0x21		/* is cascaded                      */
	call	SYM(delay)

	movw	$0xFFFB, SYM(i8259s_cache) /* set up same values in cache */
	
	jmp	SYM (_establish_stack)	# return to the bsp entry code

/*-------------------------------------------------------------------------+
|         Function: _return_to_monitor
|      Description: Return to board's monitor (we have none so simply restart).
| Global Variables: None.
|        Arguments: None.
|          Returns: Nothing. 
+--------------------------------------------------------------------------*/

	.p2align 4
	
        PUBLIC (_return_to_monitor)
SYM (_return_to_monitor):

	call	SYM (Timer_exit)
	call	SYM (Clock_exit)
	jmp	SYM (start)

/*-------------------------------------------------------------------------+
|         Function: _default_int_handler
|      Description: default interrupt handler
| Global Variables: None.
|        Arguments: None.
|          Returns: Nothing. 
+--------------------------------------------------------------------------*/
	.p2align 4
	
/*---------------------------------------------------------------------------+
| GDT itself
+--------------------------------------------------------------------------*/

	.p2align 4
		
	PUBLIC (_Global_descriptor_table)
SYM (_Global_descriptor_table):

	/* NULL segment */
	.word 0, 0     
	.byte 0, 0, 0, 0

	/* code segment */
	.word 0xffff, 0
	.byte 0, 0x9e, 0xcf, 0

	/* data segment */
	.word 0xffff, 0
	.byte 0, 0x92, 0xcf, 0
 

/*---------------------------------------------------------------------------+
| Descriptor of GDT
+--------------------------------------------------------------------------*/
SYM (gdtdesc):
	.word (3*8 - 1)  
	.long SYM (_Global_descriptor_table)


/*---------------------------------------------------------------------------+
| IDT itself
+---------------------------------------------------------------------------*/
	.p2align 4
	
	PUBLIC(Interrupt_descriptor_table)
SYM(Interrupt_descriptor_table):
	.rept 256
	.word 0,0,0,0
	.endr
	
/*---------------------------------------------------------------------------+
| Descriptor of IDT
+--------------------------------------------------------------------------*/
SYM(idtdesc):	
	.word  (256*8 - 1)
	.long  SYM (Interrupt_descriptor_table)
	
END_CODE

END

/*  bsp_start()
 *
 *  This routine starts the application.  It includes application,
 *  board, and monitor specific initialization and configuration.
 *  The generic CPU dependent initialization has been performed
 *  before this routine is invoked.
 * 
 *  INPUT:  NONE
 * 
 *  OUTPUT: NONE
 *
 *  COPYRIGHT (c) 1989, 1990, 1991, 1992, 1993, 1994.
 *  On-Line Applications Research Corporation (OAR).
 *  All rights assigned to U.S. Government, 1994.
 *
 *  This material may be reproduced by or for the U.S. Government pursuant
 *  to the copyright license under the clause at DFARS 252.227-7013.  This
 *  notice must appear in all copies of this file and its derivatives.
 *
 *  $Id$
 */

#include "rtems.h"
#include "bsp.h"
#include "cpu.h"
#include "libcsupport.h"

unsigned char *duart_base;
extern struct duart_regs duart_info;

#define DELAY 5000

void led_putnum();

/*
 *  The original table from the application and our copy of it with
 *  some changes.
 */
 
extern rtems_configuration_table  Configuration;
rtems_configuration_table         BSP_Configuration;

rtems_cpu_table Cpu_table;
 
/*      Initialize whatever libc we are using
 *      called from postdriver hook
 */
 
void bsp_libc_init()
{
    extern int end;
    rtems_unsigned32        heap_start;
 
    heap_start = (rtems_unsigned32) &end;
    if (heap_start & (CPU_ALIGNMENT-1))
        heap_start = (heap_start + CPU_ALIGNMENT) & ~(CPU_ALIGNMENT-1);
 
	/* Create 64 KByte memory region for RTEMS executive */
    RTEMS_Malloc_Initialize((void *) heap_start, 64 * 1024, 0);
 
    /*
     * Set up for the libc handling.
     */
 
    if (BSP_Configuration.ticks_per_timeslice > 0)
        libc_init(1);                /* reentrant if possible */
	else
        libc_init(0);                /* non-reentrant */
 
    /*
     *  Initialize the stack bounds checker
     */
 
#ifdef STACK_CHECKER_ON
    Stack_check_Initialize();
#endif
}
 

int bsp_start(
  int argc,
  char **argv,
  char **environp
)
{
  m68k_isr *monitors_vector_table;
  int          index;

  duart_base = (unsigned char *)DUART_ADDR;

  /* 
   *  Set the VBR here to the monitor's default.
   */

  monitors_vector_table = (m68k_isr *)0;   /* This is where
											  you set vector base
											  register = 0 */
  m68k_set_vbr( monitors_vector_table );

  /* The vector interrupt table for the 680x0 is in appendix B-2 
	 of the M68000 Family Programmer's reference table */
  for ( index=2 ; index<=255 ; index++ )
    M68Kvec[ index ] = monitors_vector_table[ 32 ];


  M68Kvec[  2 ] = monitors_vector_table[  2 ];   /* bus error vector */
  M68Kvec[  4 ] = monitors_vector_table[  4 ];   /* breakpoints vector */
  M68Kvec[  9 ] = monitors_vector_table[  9 ];   /* trace vector */

  /* 
   *  Set the VBR here if you do not want to use the monitor's vector table.
   */

  m68k_set_vbr( &M68Kvec );

  m68k_enable_caching();

  /*
   *  we only use a hook to get the C library initialized.
   */
 
  Cpu_table.pretasking_hook = NULL;
 
  Cpu_table.predriver_hook = bsp_libc_init;  /* RTEMS resources available */
 
  Cpu_table.postdriver_hook = NULL;
 
  Cpu_table.idle_task = NULL;  /* do not override system IDLE task */

  Cpu_table.do_zero_of_workspace = TRUE;
 
  Cpu_table.interrupt_vector_table = (m68k_isr *) &M68Kvec;
 
  Cpu_table.interrupt_stack_size = 4096;

  Cpu_table.extra_system_initialization_stack = 0;

  /*
   *  Copy the table
   */
 
  BSP_Configuration = Configuration;
 
  BSP_Configuration.work_space_start = (void *)
     (RAM_END - BSP_Configuration.work_space_size);
 
  /*
   * Add 1 region for the RTEMS Malloc
   */
 
  BSP_Configuration.maximum_regions++;
 
  /*
   * Add 1 extension for newlib libc
   */
 
#ifdef RTEMS_NEWLIB
    BSP_Configuration.maximum_extensions++;
#endif
 
  /*
   * Add another extension if using the stack checker
   */
 
#ifdef STACK_CHECKER_ON
    BSP_Configuration.maximum_extensions++;
#endif
 
/*  led_putnum('e'); * for debugging purposes only */
  rtems_initialize_executive( &BSP_Configuration, &Cpu_table );/* does not return */
 
  /* Clock_exit is done as an atexit() function */

  return 0;
}

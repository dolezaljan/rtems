/*  Clock_init()
 *
 *  This routine initializes the Z80386 1 on the MVME136 board.
 *  The tick frequency is 1 millisecond.
 *
 *  Input parameters:  NONE
 *
 *  Output parameters:  NONE
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

#include <stdlib.h>

#include <rtems.h>
#include <bsp.h>
#include <clockdrv.h>

rtems_unsigned32 Clock_isrs;        /* ISRs until next tick */
volatile rtems_unsigned32 Clock_driver_ticks;
                                    /* ticks since initialization */
rtems_isr_entry  Old_ticker;

rtems_device_driver Clock_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void *pargp,
  rtems_id tid,
  rtems_unsigned32 *rval
)
{
  Install_clock( Clock_isr );
}

void ReInstall_clock(
  rtems_isr_entry clock_isr
)
{
  rtems_unsigned32 isrlevel = 0 ;

  rtems_interrupt_disable( isrlevel );
   (void) set_vector( clock_isr, TIMER_VECTOR, 1 );
  rtems_interrupt_enable( isrlevel );
}

void Install_clock(
  rtems_isr_entry clock_isr
)
{
  rtems_unsigned8 data;

  Clock_driver_ticks = 0;
  Clock_isrs = BSP_Configuration.microseconds_per_tick / 1000;

  if ( BSP_Configuration.ticks_per_timeslice ) {
    Old_ticker = (rtems_isr_entry) set_vector( clock_isr, TIMER_VECTOR, 1 );

    Z8x36_WRITE( TIMER, MASTER_CFG, 0xd4 );
    Z8x36_READ ( TIMER, MASTER_INTR, data );
    Z8x36_WRITE( TIMER, MASTER_INTR, (data & 0x7E) );
    Z8x36_WRITE( TIMER, CT1_TIME_CONST_MSB, 0x04 );
    Z8x36_WRITE( TIMER, CT1_TIME_CONST_LSB, 0xCE );
    Z8x36_WRITE( TIMER, CT1_MODE_SPEC, 0x83 );
    Z8x36_WRITE( TIMER, CNT_TMR_VECTOR, TIMER_VECTOR );
    Z8x36_WRITE( TIMER, CT1_CMD_STATUS, 0x20 );
    Z8x36_READ ( TIMER, MASTER_INTR, data );
    Z8x36_WRITE( TIMER, MASTER_INTR, (data & 0xDA) | 0x80 );

    /*
     * ACC_IC54 - interrupt 5 will be vectored and mapped to level 6
     */

    data = (*(rtems_unsigned8 *)0x0D00000B);
    (*(rtems_unsigned8 *)0x0D00000B) = (data & 0x7F) | 0x60;

    Z8x36_WRITE( TIMER, CT1_CMD_STATUS, 0xC6 );

    atexit( Clock_exit );
  }
}

void Clock_exit( void )
{
  rtems_unsigned8 data;

  if ( BSP_Configuration.ticks_per_timeslice ) {

    Z8x36_READ ( TIMER, MASTER_INTR, data );
    Z8x36_WRITE( TIMER, MASTER_INTR, (data & 0x01) );
    /* do not restore old vector */

  }
}

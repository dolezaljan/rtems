/*  tod.inl
 *
 *  This file contains the static inline implementation of the inlined routines
 *  from the Time of Day Handler.
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

#ifndef __TIME_OF_DAY_inl
#define __TIME_OF_DAY_inl

/*PAGE
 *
 *  _TOD_Is_set
 *
 */

STATIC INLINE boolean _TOD_Is_set( void )
{
  return _Watchdog_Is_active( &_TOD_Seconds_watchdog );
}

/*PAGE
 *
 *  _TOD_Tickle_ticks
 *
 */

STATIC INLINE void _TOD_Tickle_ticks( void )
{
  _TOD_Current.ticks += 1;
  _TOD_Ticks_since_boot += 1;
}

/*PAGE
 *
 *  _TOD_Deactivate
 *
 */

STATIC INLINE void _TOD_Deactivate( void )
{
  _Watchdog_Remove( &_TOD_Seconds_watchdog );
}

/*PAGE
 *
 *  _TOD_Activate
 *
 */

STATIC INLINE void _TOD_Activate(
  rtems_interval ticks
)
{
  _Watchdog_Insert_ticks(
    &_TOD_Seconds_watchdog,
    ticks,
    WATCHDOG_ACTIVATE_NOW
  );
}

#endif
/* end of include file */

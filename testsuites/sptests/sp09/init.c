/*  Init
 *
 *  This routine is the initialization task for this test program.
 *  It is a user initialization task and has the responsibility for creating
 *  and starting the tasks that make up the test.  If the time of day
 *  clock is required for the test, it should also be set to a known
 *  value by this function.
 *
 *  Input parameters:
 *    argument - task argument
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

#include "system.h"
#undef EXTERN
#define EXTERN
#include "conftbl.h"
#include "gvar.h"

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  puts( "\n\n*** TEST 9 ***" );

  Task_name[ 1 ]       =  rtems_build_name( 'T', 'A', '1', ' ' );
  Task_name[ 2 ]       =  rtems_build_name( 'T', 'A', '2', ' ' );
  Task_name[ 3 ]       =  rtems_build_name( 'T', 'A', '3', ' ' );
  Task_name[ 4 ]       =  rtems_build_name( 'T', 'A', '4', ' ' );
  Task_name[ 5 ]       =  rtems_build_name( 'T', 'A', '5', ' ' );
  Task_name[ 6 ]       =  rtems_build_name( 'T', 'A', '6', ' ' );
  Task_name[ 7 ]       =  rtems_build_name( 'T', 'A', '7', ' ' );
  Task_name[ 8 ]       =  rtems_build_name( 'T', 'A', '8', ' ' );
  Task_name[ 9 ]       =  rtems_build_name( 'T', 'A', '9', ' ' );
  Task_name[ 10 ]      =  rtems_build_name( 'T', 'A', 'A', ' ' );

  Timer_name[ 1 ]      =  rtems_build_name( 'T', 'M', '1', ' ' );

  Semaphore_name[ 1 ]  =  rtems_build_name( 'S', 'M', '1', ' ' );
  Semaphore_name[ 2 ]  =  rtems_build_name( 'S', 'M', '2', ' ' );
  Semaphore_name[ 3 ]  =  rtems_build_name( 'S', 'M', '3', ' ' );

  Queue_name[ 1 ]      =  rtems_build_name( 'M', 'Q', '1', ' ' );
  Queue_name[ 2 ]      =  rtems_build_name( 'M', 'Q', '2', ' ' );

  Partition_name[ 1 ]  =  rtems_build_name( 'P', 'T', '1', ' ' );

  Region_name[ 1 ]     =  rtems_build_name( 'R', 'N', '1', ' ' );

  Port_name[ 1 ]       =  rtems_build_name( 'D', 'P', '1', ' ' );

  Period_name[ 1 ]     =  rtems_build_name( 'T', 'M', '1', ' ' );

#if 0
  status = rtems_task_create(
    Task_name[1],
    4,
    10,
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 1 ]
  );
  fatal_directive_status(
    status,
    RTEMS_INVALID_SIZE,
    "rtems_task_create with illegal stack size"
  );
  puts( "INIT - rtems_task_create - RTEMS_INVALID_SIZE" );
#endif
  puts( "INIT - rtems_task_create - RTEMS_INVALID_SIZE -- NOT CHECKED" );

  status = rtems_task_create(
     Task_name[1],
     0,
     2048,
     RTEMS_DEFAULT_MODES,
     RTEMS_DEFAULT_ATTRIBUTES,
     &Task_id[ 1 ]
  );
  fatal_directive_status(
    status,
    RTEMS_INVALID_PRIORITY,
    "rtems_task_create with illegal priority"
  );
  puts( "INIT - rtems_task_create - RTEMS_INVALID_PRIORITY" );

  status = rtems_task_create(
    Task_name[ 1 ],
    4,
    2048,
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 1 ]
  );
  directive_failed( status, "rtems_task_create of TA1" );

  status = rtems_task_restart( Task_id[ 1 ], 0 );
  fatal_directive_status(
    status,
    RTEMS_INCORRECT_STATE,
    "rtems_task_restart of DORMANT task"
  );
  puts( "INIT - rtems_task_restart - RTEMS_INCORRECT_STATE" );

  status = rtems_task_start( Task_id[ 1 ], Task_1, 0 );
  directive_failed( status, "rtems_task_start of TA1" );

  status = rtems_task_delete( RTEMS_SELF );
  directive_failed( status, "rtems_task_delete of RTEMS_SELF" );
}

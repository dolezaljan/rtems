/*  Task_1
 *
 *  This routine serves as a test task.  It verifies the semaphore manager.
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

rtems_task Task_1(
  rtems_task_argument argument
)
{
  rtems_id          smid;
  rtems_status_code status;

  status = rtems_semaphore_ident(
    Semaphore_name[ 1 ],
    RTEMS_SEARCH_ALL_NODES,
    &smid
  );
  printf( "TA1 - rtems_semaphore_ident - smid => %08x\n", smid );
  directive_failed( status, "rtems_semaphore_ident of SM1" );

  puts( "TA1 - rtems_semaphore_obtain - wait forever on SM2" );
  status = rtems_semaphore_obtain(
    Semaphore_id[ 2 ],
    RTEMS_DEFAULT_OPTIONS,
    RTEMS_NO_TIMEOUT
  );
  directive_failed( status, "rtems_semaphore_obtain of SM2" );
  puts( "TA1 - got SM2" );

  puts( "TA1 - rtems_semaphore_obtain - wait forever on SM3" );
  status = rtems_semaphore_obtain(
    Semaphore_id[ 3 ],
    RTEMS_DEFAULT_OPTIONS,
    RTEMS_NO_TIMEOUT
  );
  directive_failed( status, "rtems_semaphore_obtain of SM3" );
  puts( "TA1 - got SM3" );

  puts( "TA1 - rtems_semaphore_obtain - get SM1 - RTEMS_NO_WAIT" );
  status = rtems_semaphore_obtain(
    Semaphore_id[ 1 ],
    RTEMS_NO_WAIT,
    RTEMS_NO_TIMEOUT
  );
  directive_failed( status, "rtems_semaphore_obtain of SM1" );
  puts( "TA1 - got SM1" );

  puts( "TA1 - rtems_task_wake_after - sleep 5 seconds" );
  status = rtems_task_wake_after( 5*TICKS_PER_SECOND );
  directive_failed( status, "rtems_task_wake_after" );

pause();

  puts( "TA1 - rtems_semaphore_release - release SM1" );
  status = rtems_semaphore_release( Semaphore_id[ 1 ] );
  directive_failed( status, "rtems_semaphore_release of SM1" );

  puts(
    "TA1 - rtems_semaphore_obtain - waiting for SM1 with 10 second timeout"
  );
  status = rtems_semaphore_obtain(
    Semaphore_id[ 1 ],
    RTEMS_DEFAULT_OPTIONS,
    10 * TICKS_PER_SECOND
  );
  directive_failed( status, "rtems_semaphore_obtain of SM1" );
  puts( "TA1 - got SM1" );

  puts( "TA1 - rtems_semaphore_release - release SM2" );
  status = rtems_semaphore_release( Semaphore_id[ 2 ] );
  directive_failed( status, "rtems_semaphore_release of SM2" );

  puts( "TA1 - rtems_task_wake_after - sleep 5 seconds" );
  status = rtems_task_wake_after( 5*TICKS_PER_SECOND );
  directive_failed( status, "rtems_task_wake_after" );

pause();

  puts( "TA1 - rtems_task_delete - delete TA3" );
  status = rtems_task_delete( Task_id[ 3 ] );
  directive_failed( status, "rtems_task_delete of TA3" );

  status = rtems_task_create(
    Task_name[ 4 ],
    4,
    2048,
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 4 ]
  );
  directive_failed( status, "rtems_task_create of TA4" );

  status = rtems_task_create(
    Task_name[ 5 ],
    4,
    2048,
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 5 ]
   );
  directive_failed( status, "rtems_task_create of TA5" );

  status = rtems_task_start( Task_id[ 4 ], Task_4, 0 );
  directive_failed( status, "rtems_task_start of TA4" );

  status = rtems_task_start( Task_id[ 5 ], Task5, 0 );
  directive_failed( status, "rtems_task_start of TA5" );

  puts( "TA1 - rtems_task_wake_after - sleep 5 seconds" );
  status = rtems_task_wake_after( 5*TICKS_PER_SECOND );
  directive_failed( status, "rtems_task_wake_after" );

  puts( "TA1 - rtems_task_delete - delete TA4" );
  status = rtems_task_delete( Task_id[ 4 ] );
  directive_failed( status, "rtems_task_delete of TA4" );

  puts( "TA1 - rtems_semaphore_release - release SM1" );
  status = rtems_semaphore_release( Semaphore_id[ 1 ] );
  directive_failed( status, "rtems_semaphore_release on SM1" );

  puts( "TA1 - rtems_task_wake_after - sleep 5 seconds" );
  status = rtems_task_wake_after( 5*TICKS_PER_SECOND );
  directive_failed( status, "rtems_task_wake_after" );

  puts( "TA1 - rtems_semaphore_delete - delete SM1" );
  status = rtems_semaphore_delete( Semaphore_id[ 1 ] );
  directive_failed( status, "rtems_semaphore_delete of SM1" );

  puts( "TA1 - rtems_semaphore_delete - delete SM3" );
  status = rtems_semaphore_delete( Semaphore_id[ 3 ] );
  directive_failed( status, "rtems_semaphore_delete of SM3" );

  puts( "TA1 - rtems_task_delete - delete self" );
  status = rtems_task_delete( RTEMS_SELF );
  directive_failed( status, "rtems_task_delete of TA1" );
}

/*
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

rtems_unsigned32 Task_count;

rtems_task test_init(
  rtems_task_argument argument
);

rtems_task Middle_tasks(
  rtems_task_argument argument
);

rtems_task High_task(
  rtems_task_argument argument
);

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_id          id;
  rtems_status_code status;

  puts( "\n\n*** TIME TEST 16 ***" );

  status = rtems_task_create(
    rtems_build_name( 'T', 'E', 'S', 'T' ),
    251,
    2048,
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &id
  );
  directive_failed( status, "rtems_task_create of test_init" );

  status = rtems_task_start( id, test_init, 0 );
  directive_failed( status, "rtems_task_start of test_init" );

  status = rtems_task_delete( RTEMS_SELF );
  directive_failed( status, "rtems_task_delete of RTEMS_SELF" );
}

rtems_task test_init(
  rtems_task_argument argument
)
{
  rtems_task_priority priority;
  rtems_status_code   status;
  rtems_unsigned32    index;
  rtems_task_entry    task_entry;

/*  As each task is started, it preempts this task and
 *  performs a blocking rtems_event_receive.  Upon completion of
 *  this loop all created tasks are blocked.
 */

  priority = 250;

  for( index = 0 ; index <= OPERATION_COUNT ; index++ ) {
    status = rtems_task_create(
      rtems_build_name( 'M', 'I', 'D', ' ' ),
      priority,
      1024,
      RTEMS_DEFAULT_MODES,
      RTEMS_DEFAULT_ATTRIBUTES,
      &Task_id[ index ]
    );
    directive_failed( status, "rtems_task_create LOOP" );

    if (  index == OPERATION_COUNT ) task_entry = High_task;
    else                             task_entry = Middle_tasks;

    status = rtems_task_start( Task_id[ index ], task_entry, 0 );
    directive_failed( status, "rtems_task_start LOOP" );

    priority--;
  }

  Task_count = 0;

  Timer_initialize();
    (void) rtems_event_send( Task_id[ Task_count ], RTEMS_EVENT_16 );
  /* preempts task */
}

rtems_task Middle_tasks(
  rtems_task_argument argument
)
{
  rtems_event_set event_out;

  (void) rtems_event_receive(              /* task blocks */
           RTEMS_EVENT_16,
           RTEMS_DEFAULT_OPTIONS,
           RTEMS_NO_TIMEOUT,
           &event_out
         );

  Task_count++;

  (void) rtems_event_send(               /* preempts task */
    Task_id[ Task_count ],
    RTEMS_EVENT_16
  );
}

rtems_task High_task(
  rtems_task_argument argument
)
{
  rtems_event_set event_out;

  (void) rtems_event_receive(                /* task blocks */
            RTEMS_EVENT_16,
            RTEMS_DEFAULT_OPTIONS,
            RTEMS_NO_TIMEOUT,
            &event_out
          );

  end_time = Read_timer();

  put_time(
    "rtems_event_send (preemptive)",
    end_time,
    OPERATION_COUNT,
    0,
    CALLING_OVERHEAD_EVENT_SEND
  );

  exit( 0 );
}

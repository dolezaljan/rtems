/**
 *  @file
 *
 *  @brief Scheduler Simple Ready Queue Enqueue First
 *  @ingroup ScoreScheduler
 */

/*
 *  COPYRIGHT (c) 2011.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/score/schedulersimpleimpl.h>

void _Scheduler_simple_Ready_queue_enqueue_first(
  Scheduler_Control *scheduler_base,
  Thread_Control    *the_thread
)
{
  Scheduler_simple_Control *scheduler =
    _Scheduler_simple_Self_from_base( scheduler_base );

  _Scheduler_simple_Insert_priority_lifo( &scheduler->Ready, the_thread );
}

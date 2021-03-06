/**
 * @file
 *
 * @brief Removes Thread from Thread Queue
 *
 * @ingroup ScoreScheduler
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

#include <rtems/score/schedulerpriorityimpl.h>

void _Scheduler_priority_Extract(
  Scheduler_Control *base,
  Thread_Control    *the_thread
)
{
  _Scheduler_priority_Extract_body( base, the_thread );
}

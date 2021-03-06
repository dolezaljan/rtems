/**
 * @file
 *
 * @brief Priority Scheduler Schedule Method
 * @ingroup ScoreScheduler
 */
 
/*
 *  Copyright (C) 2010 Gedare Bloom.
 *  Copyright (C) 2011 On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/score/schedulerpriorityimpl.h>

void _Scheduler_priority_Schedule(
  Scheduler_Control *base,
  Thread_Control    *the_thread
)
{
  _Scheduler_priority_Schedule_body( base, the_thread, false );
}

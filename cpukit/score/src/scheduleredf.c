/**
 * @file
 *
 * @brief Scheduler EDF Initialize and Support
 * @ingroup ScoreScheduler
 */

/*
 *  Copyright (C) 2011 Petr Benes.
 *  Copyright (C) 2011 On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/score/scheduleredf.h>
#include <rtems/score/schedulerimpl.h>
#include <rtems/score/thread.h>
#include <rtems/score/wkspace.h>

static int _Scheduler_EDF_RBTree_compare_function
(
  const RBTree_Node* n1,
  const RBTree_Node* n2
)
{
  Priority_Control value1 = _RBTree_Container_of
    (n1,Scheduler_EDF_Per_thread,Node)->thread->current_priority;
  Priority_Control value2 = _RBTree_Container_of
    (n2,Scheduler_EDF_Per_thread,Node)->thread->current_priority;

  /*
   * This function compares only numbers for the red-black tree,
   * but priorities have an opposite sense.
   */
  return (-1)*_Scheduler_EDF_Priority_compare(value1, value2);
}

void _Scheduler_EDF_Initialize(void)
{
  Scheduler_EDF_Control *scheduler =
    _Workspace_Allocate_or_fatal_error( sizeof( *scheduler ) );

  _RBTree_Initialize_empty(
    &scheduler->Ready,
    _Scheduler_EDF_RBTree_compare_function,
    0
  );

  _Scheduler.information = scheduler;
}

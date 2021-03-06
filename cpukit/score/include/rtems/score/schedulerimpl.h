/**
 * @file
 *
 * @brief Inlined Routines Associated with the Manipulation of the Scheduler
 *
 * This inline file contains all of the inlined routines associated with
 * the manipulation of the scheduler.
 */

/*
 *  Copyright (C) 2010 Gedare Bloom.
 *  Copyright (C) 2011 On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_SCORE_SCHEDULERIMPL_H
#define _RTEMS_SCORE_SCHEDULERIMPL_H

#include <rtems/score/scheduler.h>
#include <rtems/score/threadimpl.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup ScoreScheduler
 */
/**@{**/

/**
 *  @brief Initializes the scheduler to the policy chosen by the user.
 *
 *  This routine initializes the scheduler to the policy chosen by the user
 *  through confdefs, or to the priority scheduler with ready chains by
 *  default.
 */
void _Scheduler_Handler_initialization( void );

/**
 * The preferred method to add a new scheduler is to define the jump table
 * entries and add a case to the _Scheduler_Initialize routine.
 *
 * Generic scheduling implementations that rely on the ready queue only can
 * be found in the _Scheduler_queue_XXX functions.
 */

/*
 * Passing the Scheduler_Control* to these functions allows for multiple
 * scheduler's to exist simultaneously, which could be useful on an SMP
 * system.  Then remote Schedulers may be accessible.  How to protect such
 * accesses remains an open problem.
 */

/**
 * @brief Scheduler schedule.
 *
 * This kernel routine implements the scheduling decision logic for
 * the scheduler. It does NOT dispatch.
 *
 * @param[in] the_thread The thread which state changed previously.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Schedule(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.schedule )( scheduler, the_thread );
}

/**
 * @brief Scheduler yield with a particular thread.
 *
 * This routine is invoked when a thread wishes to voluntarily transfer control
 * of the processor to another thread.
 *
 * @param[in] the_thread The yielding thread.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Yield(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.yield )( scheduler, the_thread );
}

/**
 * @brief Scheduler block.
 *
 * This routine removes @a the_thread from the scheduling decision for
 * the scheduler. The primary task is to remove the thread from the
 * ready queue.  It performs any necessary schedulering operations
 * including the selection of a new heir thread.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Block(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.block )( scheduler, the_thread );
}

/**
 * @brief Scheduler unblock.
 *
 * This routine adds @a the_thread to the scheduling decision for
 * the scheduler.  The primary task is to add the thread to the
 * ready queue per the schedulering policy and update any appropriate
 * scheduling variables, for example the heir thread.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Unblock(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.unblock )( scheduler, the_thread );
}

/**
 * @brief Scheduler allocate.
 *
 * This routine allocates @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void* _Scheduler_Allocate(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  return ( *scheduler->Operations.allocate )( scheduler, the_thread );
}

/**
 * @brief Scheduler free.
 *
 * This routine frees @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Free(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.free )( scheduler, the_thread );
}

/**
 * @brief Scheduler update.
 *
 * This routine updates @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Update(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.update )( scheduler, the_thread );
}

/**
 * @brief Scheduler enqueue.
 *
 * This routine enqueue @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Enqueue(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.enqueue )( scheduler, the_thread );
}

/**
 * @brief Scheduler enqueue first.
 *
 * This routine enqueue_first @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Enqueue_first(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.enqueue_first )( scheduler, the_thread );
}

/**
 * @brief Scheduler extract.
 *
 * This routine extract @a the_thread->scheduler
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Extract(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread
)
{
  ( *scheduler->Operations.extract )( scheduler, the_thread );
}

/**
 * @brief Scheduler priority compare.
 *
 * This routine compares two priorities.
 */
RTEMS_INLINE_ROUTINE int _Scheduler_Priority_compare(
  Scheduler_Control *scheduler,
  Priority_Control p1,
  Priority_Control p2
)
{
  return ( *scheduler->Operations.priority_compare )( p1, p2 );
}

/**
 * @brief Scheduler release job.
 *
 * This routine is called when a new period of task is issued.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Release_job(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread,
  uint32_t           length
)
{
  ( *scheduler->Operations.release_job )( scheduler, the_thread, length );
}

/**
 * @brief Scheduler method invoked at each clock tick.
 *
 * This method is invoked at each clock tick to allow the scheduler
 * implementation to perform any activities required.  For the
 * scheduler which support standard RTEMS features, this includes
 * time-slicing management.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Tick( Scheduler_Control *scheduler )
{
  ( *scheduler->Operations.tick )( scheduler );
}

/**
 * @brief Starts the idle thread for a particular processor.
 *
 * @param[in,out] the_thread The idle thread for the processor.
 * @parma[in,out] processor The processor for the idle thread.
 *
 * @see _Thread_Create_idle().
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Start_idle(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread,
  Per_CPU_Control   *cpu
)
{
  ( *scheduler->Operations.start_idle )( scheduler, the_thread, cpu );
}

#if defined(__RTEMS_HAVE_SYS_CPUSET_H__) && defined(RTEMS_SMP)
  /**
   * @brief Obtain the processor affinity for a thread.
   *
   * @param[in,out] thread The thread.
   * @parma[out] cpuset The processor affinity for this thread
   */
  RTEMS_INLINE_ROUTINE int _Scheduler_Get_affinity(
    Scheduler_Control *scheduler,
    Thread_Control    *thread,
    size_t             cpusetsize,
    cpu_set_t         *cpuset
  )
  {
    return ( *scheduler->Operations.get_affinity )(
      scheduler,
      thread,
      cpusetsize,
      cpuset
    );
  }

  /**
   * @brief Set the processor affinity for a thread.
   *
   * @param[in,out] thread The thread.
   * @parma[in] cpuset The processor affinity for this thread
   */
  RTEMS_INLINE_ROUTINE int _Scheduler_Set_affinity(
    Scheduler_Control *scheduler,
    Thread_Control    *thread,
    size_t             cpusetsize,
    const cpu_set_t   *cpuset
  )
  {
    return ( *scheduler->Operations.set_affinity )(
      scheduler,
      thread,
      cpusetsize,
      cpuset
    );
  }
#endif

RTEMS_INLINE_ROUTINE void _Scheduler_Update_heir(
  Thread_Control *heir,
  bool force_dispatch
)
{
  Thread_Control *executing = _Thread_Executing;

  _Thread_Heir = heir;

  if ( executing != heir && ( force_dispatch || executing->is_preemptible ) )
    _Thread_Dispatch_necessary = true;
}

RTEMS_INLINE_ROUTINE void _Scheduler_Generic_block(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread,
  void            ( *extract )( Scheduler_Control *, Thread_Control * ),
  void            ( *schedule )( Scheduler_Control *, Thread_Control *, bool )
)
{
  ( *extract )( scheduler, the_thread );

  /* TODO: flash critical section? */

  if ( _Thread_Is_executing( the_thread ) || _Thread_Is_heir( the_thread ) ) {
    ( *schedule )( scheduler, the_thread, true );
  }
}

/**
 * @brief Returns true if @p1 encodes a lower priority than @a p2 in the
 * intuitive sense of priority.
 */
RTEMS_INLINE_ROUTINE bool _Scheduler_Is_priority_lower_than(
  Scheduler_Control *scheduler,
  Priority_Control   p1,
  Priority_Control   p2
)
{
  return _Scheduler_Priority_compare( scheduler, p1,  p2 ) < 0;
}

/**
 * @brief Returns true if @p1 encodes a higher priority than @a p2 in the
 * intuitive sense of priority.
 */
RTEMS_INLINE_ROUTINE bool _Scheduler_Is_priority_higher_than(
  Scheduler_Control *scheduler,
  Priority_Control   p1,
  Priority_Control   p2
)
{
  return _Scheduler_Priority_compare( scheduler, p1,  p2 ) > 0;
}

/**
 * @brief Returns the priority encoding @a p1 or @a p2 with the higher priority
 * in the intuitive sense of priority.
 */
RTEMS_INLINE_ROUTINE Priority_Control _Scheduler_Highest_priority_of_two(
  Scheduler_Control *scheduler,
  Priority_Control   p1,
  Priority_Control   p2
)
{
  return _Scheduler_Is_priority_higher_than( scheduler, p1, p2 ) ? p1 : p2;
}

/**
 * @brief Sets the thread priority to @a priority if it is higher than the
 * current priority of the thread in the intuitive sense of priority.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Set_priority_if_higher(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread,
  Priority_Control   priority
)
{
  Priority_Control current = the_thread->current_priority;

  if ( _Scheduler_Is_priority_higher_than( scheduler, priority, current ) ) {
    _Thread_Set_priority( the_thread, priority );
  }
}

/**
 * @brief Changes the thread priority to @a priority if it is higher than the
 * current priority of the thread in the intuitive sense of priority.
 */
RTEMS_INLINE_ROUTINE void _Scheduler_Change_priority_if_higher(
  Scheduler_Control *scheduler,
  Thread_Control    *the_thread,
  Priority_Control   priority,
  bool               prepend_it
)
{
  Priority_Control current = the_thread->current_priority;

  if ( _Scheduler_Is_priority_higher_than( scheduler, priority, current ) ) {
    _Thread_Change_priority( the_thread, priority, prepend_it );
  }
}

RTEMS_INLINE_ROUTINE Scheduler_Control *_Scheduler_Get(
  Thread_Control *the_thread
)
{
  (void) the_thread;

  return &_Scheduler;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif
/* end of include file */

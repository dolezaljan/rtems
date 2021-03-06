/**
 * @file
 *
 * @ingroup ScoreProfiling
 *
 * @brief Profiling Support API
 */

/*
 * Copyright (c) 2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_SCORE_PROFILING
#define _RTEMS_SCORE_PROFILING

#include <rtems/score/percpu.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup ScoreProfiling Profiling Support
 *
 * @brief Profiling support.
 *
 * @{
 */

static inline void _Profiling_Thread_dispatch_disable(
  Per_CPU_Control *per_cpu,
  uint32_t previous_thread_dispatch_disable_level
)
{
#if defined( RTEMS_PROFILING )
  if ( previous_thread_dispatch_disable_level == 0 ) {
    Per_CPU_Stats *stats = &per_cpu->Stats;

    stats->thread_dispatch_disabled_instant = _CPU_Counter_read();
    ++stats->thread_dispatch_disabled_count;
  }
#else
  (void) per_cpu;
  (void) previous_thread_dispatch_disable_level;
#endif
}

static inline void _Profiling_Thread_dispatch_enable(
  Per_CPU_Control *per_cpu,
  uint32_t new_thread_dispatch_disable_level
)
{
#if defined( RTEMS_PROFILING )
  if ( new_thread_dispatch_disable_level == 0 ) {
    Per_CPU_Stats *stats = &per_cpu->Stats;
    CPU_Counter_ticks now = _CPU_Counter_read();
    CPU_Counter_ticks delta = _CPU_Counter_difference(
      now,
      stats->thread_dispatch_disabled_instant
    );

    stats->total_thread_dispatch_disabled_time += delta;

    if ( stats->max_thread_dispatch_disabled_time < delta ) {
      stats->max_thread_dispatch_disabled_time = delta;
    }
  }
#else
  (void) per_cpu;
  (void) new_thread_dispatch_disable_level;
#endif
}

static inline void _Profiling_Update_max_interrupt_delay(
  Per_CPU_Control *per_cpu,
  CPU_Counter_ticks interrupt_delay
)
{
#if defined( RTEMS_PROFILING )
  Per_CPU_Stats *stats = &per_cpu->Stats;

  if ( stats->max_interrupt_delay < interrupt_delay ) {
    stats->max_interrupt_delay = interrupt_delay;
  }
#else
  (void) per_cpu;
  (void) interrupt_delay;
#endif
}

void _Profiling_Outer_most_interrupt_entry_and_exit(
  Per_CPU_Control *per_cpu,
  CPU_Counter_ticks interrupt_entry_instant,
  CPU_Counter_ticks interrupt_exit_instant
);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RTEMS_SCORE_PROFILING */

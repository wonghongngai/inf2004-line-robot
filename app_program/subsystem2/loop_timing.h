/*
 * loop_timing.h
 * Late-cycle counter for the motion task (S1-U17, S2-U12).
 *
 * The motion loop wakes every S2_ENCODER_POLL_MS and runs a control step
 * once S2_CONTROL_PERIOD_MS has passed since the last one, so a normal
 * control gap is 10-11 ms. A step is LATE when its gap is more than
 * S2_CONTROL_LATE_MS past the period (over 15 ms): half a period lost to
 * another task. task_motion.c prints the counts in its 1 s status line:
 *
 *     ctrl late=0/1200 gap_max=11 loop_max=2 ms
 *
 * late is since boot (late steps / all steps); gap_max and loop_max are the
 * worst control gap and the worst loop wake-up gap in the last second.
 *
 * Plain C with no RTOS calls, so the host test can include it.
 */
#ifndef LOOP_TIMING_H
#define LOOP_TIMING_H

#include <stdint.h>

#ifndef S2_CONTROL_LATE_MS
#define S2_CONTROL_LATE_MS 5U
#endif

typedef struct
{
    uint32_t steps;       /* control steps since boot */
    uint32_t late;        /* of which late */
    uint32_t gap_max_ms;  /* worst control gap, this window */
    uint32_t loop_max_ms; /* worst gap between loop wake-ups, this window */
    uint32_t last_loop_ms;
    uint8_t  started;
} LOOP_TIMING;

/* Call once per loop wake-up with the current time. */
static inline void loop_timing_wake(LOOP_TIMING * p_timing, uint32_t now_ms)
{
    if (p_timing->started)
    {
        uint32_t gap = now_ms - p_timing->last_loop_ms;
        if (gap > p_timing->loop_max_ms)
        {
            p_timing->loop_max_ms = gap;
        }
    }
    p_timing->last_loop_ms = now_ms;
    p_timing->started      = 1u;
}

/* Call once per control step with the time since the previous step. */
static inline void loop_timing_control(LOOP_TIMING * p_timing, uint32_t gap_ms,
                                       uint32_t period_ms)
{
    ++p_timing->steps;
    if (gap_ms > period_ms + S2_CONTROL_LATE_MS)
    {
        ++p_timing->late;
    }
    if (gap_ms > p_timing->gap_max_ms)
    {
        p_timing->gap_max_ms = gap_ms;
    }
}

/* Call after printing: the maxima restart, the totals carry on. */
static inline void loop_timing_new_window(LOOP_TIMING * p_timing)
{
    p_timing->gap_max_ms  = 0u;
    p_timing->loop_max_ms = 0u;
}

#endif /* LOOP_TIMING_H */

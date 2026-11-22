/* Host test for loop_timing.h, the motion task's late-cycle counter
 * (S1-U17, S2-U12): what counts as late, and what each window keeps. */
#include <assert.h>
#include <stdio.h>
#include "loop_timing.h"

int main(void)
{
    LOOP_TIMING t = {0};

    /* Normal 10-11 ms gaps, and exactly period + slack (15 ms), are on time. */
    loop_timing_control(&t, 10u, 10u);
    loop_timing_control(&t, 11u, 10u);
    loop_timing_control(&t, 15u, 10u);
    assert(t.steps == 3u && t.late == 0u && t.gap_max_ms == 15u);

    /* 16 ms is late; so is a 40 ms stall. */
    loop_timing_control(&t, 16u, 10u);
    loop_timing_control(&t, 40u, 10u);
    assert(t.steps == 5u && t.late == 2u && t.gap_max_ms == 40u);

    /* The first wake-up only sets the reference; later ones track the max. */
    loop_timing_wake(&t, 1000u);
    assert(t.loop_max_ms == 0u);
    loop_timing_wake(&t, 1001u);
    loop_timing_wake(&t, 1004u);
    loop_timing_wake(&t, 1005u);
    assert(t.loop_max_ms == 3u);

    /* uint32 ms wrap-around still gives the right gap. */
    loop_timing_wake(&t, 0xFFFFFFFEu);
    loop_timing_new_window(&t);
    loop_timing_wake(&t, 0x00000003u);
    assert(t.loop_max_ms == 5u);

    /* A new window clears the maxima only. */
    loop_timing_new_window(&t);
    assert(t.gap_max_ms == 0u && t.loop_max_ms == 0u);
    assert(t.steps == 5u && t.late == 2u);

    puts("PASS: late = gap over period + 5 ms; maxima per window, totals kept; wrap-safe");
    return 0;
}

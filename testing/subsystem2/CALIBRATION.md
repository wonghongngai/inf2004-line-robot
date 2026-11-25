# Physical calibration — 2026-10-29

> **Measurement record.** The current values are in
> [subsystem2.h](../../app_program/subsystem2/subsystem2.h). The first
> measurements below are the 29 Oct polling-decoder figures. Current
> travel scales are 116.2 ticks/cm (left) and 118.3 ticks/cm (right).
> Driven pivot scales are 24.2 ticks/degree for a right pivot and 25.0
> ticks/degree for a left pivot.

The current straight-driving PID gains are Kp 6.0, Ki 0.5 and Kd 0. See
[S2-U10](TEST_LOG.md#results) for the physical tuning result. The shared
minimum effective PWM is 15% for the motion profile.

## Distance

Each run was 100 cm. Use these direct travel measurements for ticks/cm:

| Run | Left ticks | Right ticks |
| --- | ---: | ---: |
| 1 | 10633 | 11877 |
| 2 | 10166 | 11768 |
| 3 | 10259 | 11924 |
| Mean | 10352.6667 | 11856.3333 |
| Configured ticks/cm | 103.5267 | 118.5634 |

The right configured value preserves the user's reported rounding. Both
`all_test` odometry and normal integrated motion read these constants;
forward travel can now supply distance to Buddy 4's height estimator.

## One-wheel pivots

Each turn measurement was 90 degrees. Preserve all three trials in the
user's means, including the lower first right-turn result:

| Chassis turn | Moving encoder | Trial ticks | Mean ticks | Configured ticks/degree |
| --- | --- | --- | ---: | ---: |
| Right | Left | 1437, 2029, 2030 | 1832 | 20.3556 |
| Left | Right | 1950, 2086, 2149 | 2061.6667 | 22.9074 |

`S2_LEFT_TURN_TICKS_PER_DEGREE` refers to the left wheel during a right
pivot; `S2_RIGHT_TURN_TICKS_PER_DEGREE` refers to the right wheel during a
left pivot. These scales assume the stationary-wheel pivot used by this
controller, not a turn with both wheels counter-rotating.

## Measurements requiring follow-up

The separate cumulative five-rotation logs ended at 12397 left ticks and
12404 right ticks: 2479.4 and 2480.8 ticks/rotation. The three ten-rotation
trials averaged 2381.8 left and 2454.1333 right ticks/rotation. These are
recorded observations, not substituted for the direct travel calibration.

Five-rotation travel of 41.6, 41.5 and 41 cm implies a mean circumference
of 8.2733 cm, which differs from the supplied 7.854 cm circumference. These
rotation/travel results also do not reproduce the 100 cm ticks/cm values.
Confirm that rotation counts refer to the same wheel/shaft and that the
distance was measured with the same encoder firmware and sampling speed.

Two-bit jumps reached 120 left and 124 right in the cumulative logs. The
polling decoder rejects these transitions rather than inventing direction;
missing intermediate edges can make measured ticks depend on wheel speed.
The first right-turn trial (1437) is also substantially lower than 2029/2030.
Repeat distance and turn checks at the intended motor speed before relying
on precise distance, angle or hump-height estimates. The measured values
have been applied as requested without silently rejecting trials.

## Hardware validation

With wheels raised, first confirm motor direction and encoder response.
Then measure actual travel for a commanded distance and actual angle for
each 90-degree pivot. Check the average wheel-distance telemetry against
the floor measurement. Validate a completed hump against a known height;
tilting the IMU while stationary provides no wheel distance.

Motor calibration does not enable line calibration or change the normal
integrated profile's sensor layout. Those remain separate configuration
choices.

## Re-measurement 2026-11-22 (S2-U06, #18)

Hand pushes of about 100 cm (approximate distance), `robot-motion.uf2`, one push per boot.
Four pushes; run 2 is excluded because the right encoder froze mid-push.

| Run | Left ticks | Right ticks |
| --- | ---: | ---: |
| 1 | 11051 | 11302 |
| 3 | 11474 | 10825 |
| 4 | 11432 | 11492 |
| Mean | 11319 | 11206 |
| Ticks/cm | 113.2 | 112.1 |

Spread (max − min) / mean: left 3.7 %, right 6.0 %. Run 3 curved (left 6 % ahead).
These were 4–5 % below the then-current `S2_ENC_LEFT/RIGHT_TICKS_PER_CM` (117.8 / 118.3),
which were trimmed on the track for drift on 8 Nov. **`subsystem2.h` is not
changed**: a hand push is not accurate enough to override a track trim. The old
`S2_LEFT/RIGHT_TICKS_PER_CM` (23.10 / 18.83) are far from every measurement and
look obsolete.

### Pivots, 2026-11-22 (S2-U07)

Hand pivots about the stationary wheel (motors off), one 360° pivot each way,
each ending on the 360° floor mark, plus four 90° pivots that reached 90°.

| Pivot | Moving wheel | 360° ticks | Ticks/degree | 90° ticks |
| --- | --- | ---: | ---: | --- |
| Right | Left | 8287 | 23.02 | 2046, 2046 (22.7) |
| Left | Right | 8343 | 23.18 | 2073, 1951, 1941 (21.6–23.0) |

`subsystem2.h` now uses the 360° figures: `S2_LEFT_TURN_TICKS_PER_DEGREE` 23.02
(was 20.3556) and `S2_RIGHT_TURN_TICKS_PER_DEGREE` 23.18 (was 22.9074). A long
turn averages out the hand error, so it is preferred to the 90° pivots. The
old right-turn value would have turned the robot about 407° on a commanded 360°.
The 90° runs spread by about 7 % (hand pivots). Not yet checked with the
motors driving the pivot; S2-U09 does that on the track.

### Driven pivots, 2026-11-22 (S2-U07 re-check)

Hand pivots gave 23.0 / 23.2 ticks per degree, but with the motors driving the
pivot (25 % PWM, floor, `PIVOT_BENCH=1`) the robot came up short: about 5° on
the 90° pivots (2044 ticks for about 85°, 24.05) and about 20° on a 360° pivot
(8246 ticks for about 340°, 24.25). The tester's angles are by eye. A driven
wheel scrubs on the floor, so it counts more ticks per real degree than a hand
pivot. `subsystem2.h` now uses **24.2** for both pivots. The same pivot numbers
feed the integrated build's junction pivots, which run at 12.5 / 15 %, so
re-check a junction turn on the track too.

### Driven pivots, second run, 2026-11-22 (24.2 for both)

Observed by eye, no serial log kept for this run (the monitor was not attached).
Right pivots (left wheel, 24.2) came out exactly 90° and 360°. Left pivots
(right wheel, 24.2) came out short: about 5° at 90° and about 10° at 360°.
At 24.2 a left pivot stops after about 2150 ticks (90°) and 8690 ticks (360°),
so those ticks covered about 85° and 350°: 25.3 and 24.8 ticks per degree.
`S2_RIGHT_TURN_TICKS_PER_DEGREE` is now **25.0**; `S2_LEFT_TURN_TICKS_PER_DEGREE`
stays 24.2. The subsequent driven-pivot run is recorded as a [pass under S2-U07](TEST_LOG.md#results).


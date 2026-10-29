# Physical calibration — 2026-10-29

Values supplied by the user and entered in `subsystem2.h`. Motion calibration
is enabled. PID gains remain zero pending physical tuning. The combined
`all_test` profile retains its 30% base / 15% inner-wheel PWM; normal motion
retains its existing 55% base PWM. The shared minimum effective PWM is now
15%, covering the reported right-wheel reliable starting range of 10–15%.
These are PWM duty cycles, not calibrated speeds.

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
choices. `all_test` still uses the three-sensor TRACK layout and stops at
junctions rather than executing barcode turns.

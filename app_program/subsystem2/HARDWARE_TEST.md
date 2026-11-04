# Subsystem 2 motor bench test

`subsystem2_test.c` tests motion logic with simulated encoder counts. It does
not move the robot. The separate `subsystem2_hw_test.c` drives the existing
motor GPIO driver without distance/angle calibration or PID.

To check all three subsystems together, build `ROBOT_MODE=integrated_motor_test`.
This runs the same timed wheel pulses with S3 IR and S4 IMU tasks active and
prints sensor snapshots between stages. Keep wheels raised; IR decisions and
automatic IMU stops are not used for these timed diagnostic pulses. See
[INTEGRATION.md](INTEGRATION.md) for expected diagnostic lines.

The PWM reset-release bug has been fixed: PWM is now released from reset even
with the project's `USE_PTMR=0`. Rebuild and reflash; older UF2s do not contain
this fix. Startup must print `[MOTOR] PWM reset_done=1 counters L=1 R=1 check=0`.
The motor PWM divider is also explicitly set for about 10 kHz instead of the
old ~125 kHz reset default. Rebuild both standalone and integrated images.

## Connections used by the current driver

| Function | GPIO |
| --- | --- |
| Motor 1 / assumed left, M1A forward PWM | GP8 |
| Motor 1 / assumed left, M1B held LOW | GP9 |
| Motor 2 / assumed right, M2A forward PWM | GP10 |
| Motor 2 / assumed right, M2B held LOW | GP11 |
| Left encoder A / B | GP26 / GP6 |
| Right encoder A / B | GP2 / GP3 |

These are GPIO numbers, not physical header pin numbers. This table reflects
the repository driver mapping; check your board and motor connections match.
Use the Robo Pico motor terminals and its motor power supply. The code assumes
M1 drives the left wheel and M2 drives the right wheel. Actual forward direction
depends on motor wiring.

## Run

1. Raise the wheels clear of the ground and keep the motor power switch within
   reach. The sequence starts automatically on each boot of this test firmware.
2. Leave `S2_CALIBRATION_READY` at `0`; this test does not need calibration.
   Select the hardware profile on the build command line.
3. From `build_make`, rebuild and flash the resulting UF2 using the project's
   usual toolchain and BOOTSEL procedure:

   ```sh
   make ROBOT_MODE=motor_test -j8
   ```

   `TEST=1` is not required. That option runs the separate software self-tests.
   The build automatically includes the new C file.
4. Open USB serial (115200 as in the project README). After a 5-second delay,
   the test runs left wheel, right wheel, both wheels, left pivot, right pivot.
   Each stage uses 40% PWM for 750 ms, then zero PWM for at least 1500 ms.
   With the wheels raised, verify the wheel pattern rather than chassis turns.
5. Check that each expected wheel spins forward and stops between stages.
   Inspect the raw encoder counts printed after each stage: an active wheel
   should normally produce changing counts; a stationary wheel should stay near
   zero. Counts can be negative because of encoder wiring direction.
6. After the sequence, motor outputs stay zero. Power-cycle to repeat.
   Build and flash `ROBOT_MODE=motion` or `ROBOT_MODE=integrated` afterwards.

This is a manual hardware check: finishing the sequence does not prove that
motors physically moved. If a wheel runs backwards, power off and correct its
motor wiring or deliberately change its forward/reverse mapping in
`drv_motor.c`. If the sides are swapped, correct the motor connections/mapping.
If neither moves, check motor supply/switch and connections first. PWM may need
tuning for your motor; adjust `S2_HW_TEST_PWM_PERCENT` in the test header.

The bench profile starts only subsystem 2. Movement requests are rejected and normal motion status is not
updated during this test. Use the `[S2 HW]` console output and observation.
Encoder sampling still uses a 1 ms requested sleep (about 2 ms minimum with
the kernel's timer rounding, plus scheduling), so missed edges are
possible; these counts alone do not establish accurate calibration.

## Calibration for normal movement commands

Normal motion needs real values in `subsystem2.h`:

- `S2_LEFT_TICKS_PER_CM` and `S2_RIGHT_TICKS_PER_CM`: measured encoder ticks
  divided by measured distance for each wheel.
- `S2_LEFT_TURN_TICKS_PER_DEGREE`: left-wheel ticks per degree during a right
  pivot; `S2_RIGHT_TURN_TICKS_PER_DEGREE`: right-wheel ticks per degree during
  a left pivot.
- PID gains and effective PWM settings tuned on the physical robot.

Once the distance/turn measurements are valid, set `S2_CALIBRATION_READY` to
`1`. All PID gains currently default to zero, which gives no straight-line
correction until tuned. The timed bench pulses verify wiring and motor output;
they do not verify centimetres travelled, turn angles, or PID performance.

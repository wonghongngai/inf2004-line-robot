# Buddy 2 straight-line PID tuning report

**Test** S2-U10 · **Test date** 22 November 2026 · **Tester** Hong Ngai

The tuned controller reduced the encoder-measured difference in wheel travel
to within **3 mm from 300 ms onward** during one commanded 100 cm floor run.
The mismatch ended at **1 mm**, meeting the **±10 mm** wheel-travel criterion.
The three zero-gain baseline runs ended with mismatches of **94–126 mm**.
[S2-U10 test record](TEST_LOG.md#results)

## Controller and gains

For a standard forward distance move, the controller converts each wheel's
encoder count change into travel using its own ticks-per-centimetre scale.
It computes the error as left wheel travel minus right wheel travel. A
positive error means the left wheel is ahead, so the controller subtracts
the correction from left PWM and adds it to right PWM.

| Setting | Baseline | Tuned and current |
|---|---:|---:|
| Proportional gain, Kp | 0 | 6.0 |
| Integral gain, Ki | 0 | 0.5 |
| Derivative gain, Kd | 0 | 0 |

The proportional term responds to the current travel mismatch and the
integral term accumulates persistent mismatch. With Kd set to zero, the
tuned controller uses proportional and integral correction. The bench
gains were later promoted to the shared `S2_PID_*` constants.
[Motion controller](../../app_program/subsystem2/motion_control.c) · [Firmware settings](../../app_program/subsystem2/subsystem2.h)

## Floor test

The S2-only bench commanded a **100 cm forward move** on the floor at
**25% base PWM**. Each run started with at least 1.2 m of clear floor. The
bench logged both encoder counts, left-minus-right travel in millimetres
(`err_mm`), and both PWM outputs every 20 ms. The archived excerpts retain
samples at about 100 ms intervals.

```sh
sh build.sh motion MOTION_DEMO=1 PID_BENCH=1
```

The first move in each boot is the S2-U10 trial. The second move checks STOP
latency under S2-U11. The S2-U10 criterion is an absolute travel error of at
most 10 mm from 300 ms to the end, with the correction remaining at its
limit for no more than 300 ms. The test log identifies the bench firmware
as `849bc09` with local PID bench changes.
[Bench procedure](HARDWARE_TEST.md#floor-bench-for-s2-u10-and-s2-u11-18) · [Test definition and results](TEST_LOG.md)

## Results

| Run | Gains (Kp / Ki / Kd) | End error reported in the test log |
|---|---|---:|
| Baseline 1 | 0 / 0 / 0 | 126 mm |
| Baseline 2 | 0 / 0 / 0 | 99 mm |
| Baseline 3 | 0 / 0 / 0 | 94 mm |
| Tuned 1 | 6.0 / 0.5 / 0 | 1 mm |

The tuned run lasted about 6.1 s. The retained samples show the left PWM at
23–25% and the right PWM at 25–26%, consistent with correcting the left
wheel's lead. The test log records a pass for this run. The retained
100 ms samples support the reported error bound and show small PWM changes.
Assessing correction saturation throughout the run requires the full-rate
trace. [Baseline trace](logs/2026-11-22_S2-U10-U11_baseline.txt) · [Tuned trace](logs/2026-11-22_S2-U10_tuned.txt)

## Scope of the result

This test measures wheel-travel mismatch inferred from the encoders during
a fixed-distance move. S2-U08 measures lateral offset on the guide line,
where the robot uses separate line steering and encoder trim. The available
evidence supports the selected gains for the tested floor run at 25% base
PWM. Repeatability across tuned runs and performance at other PWM settings
remain open. [S2-U08 correction](TEST_LOG.md#results)

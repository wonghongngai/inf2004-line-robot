# Buddy 4: IMU, humps, tilt (subsystem 4)

The IMU task reads the LSM303DLHC (accelerometer + magnetometer) every 20 ms,
calibrates a baseline at start-up, detects tilt and humps, estimates the
highest hump, and publishes a snapshot for the Vehicle Controller and
telemetry.

| File | What it does |
|---|---|
| `drv_lsm303dlhc.c/.h` | I2C driver for the LSM303DLHC |
| `task_imu.c` | IMU task: calibration, tilt/hump detection, hump height estimate |
| `subsystem4.c/.h` | Kernel objects, `subsystem4_get_latest()` snapshot, 4-byte packed message, distance input from S2 |
| `subsystem4_test.c` | On-Pico checks S4-U01 to S4-U07 (`make TEST=1`) |

## Wiring and configuration

- I2C0 on **GP4 (SDA)** and **GP5 (SCL)**. Already enabled in
  `config/config_device.h` (`DEVCNF_USE_IIC 1`) and
  `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`.
- Mount the IMU with its X axis pointing forward.
- Keep the robot **stationary for the first second** after boot: that is when
  the baseline is calibrated.

## Status (reviewed 25 Nov 2026)

- **Hump thresholds tuned 23 Nov (#27)** from `S4_TRACE` captures of the 10 mm
  one-side ramp: every decision uses the 200 ms (10-sample) average of the
  forward axis; a hump starts when it holds >= 30 mg for 200 ms while the
  wheels turn, the climb is counted from the last level sample, and it ends
  after 200 ms within +-20 mg (timeout 5 s). The height factor is
  4/3, refitted on the robot from eight climbs (#27 step 5); no hump may start
  for 500 ms after the wheels start from a standstill. The tilt
  threshold `S4_TILT_DELTA_MG` is still a placeholder (S4-U08, issue #25).
- Per [decision #5](../../DECISIONS.md#5-hump-height-after-the-descent),
  climb height is retained after descent and the highest hump is reported.
  The host test no longer expects zero after descent. S4-U09/U10 passed
  on the one-side 10 mm ramp on 23 Nov; see the
  [ramp results](../../testing/subsystem4/TEST_LOG.md#results).
- Collision detection was **removed** on the instructor's instructions
  (13 Nov) and replaced by a lift-stop (#53, 21 Nov). Lift = the 200 ms
  average of the Z deviation from `baseline_az_mg` (each sample clipped to
  ±`S4_LIFT_CLIP_MG`) stays at ≥ `S4_LIFT_DELTA_MG` (200 mg) for 3 samples.
  S4 then reports `IMU_MOTION_LIFTED` every sample and `IMU_EVENT_LIFT` once
  (code 3, which was COLLISION); the VC and `all_test` stop and stay stopped
  until reboot. The 23 Nov S4-U12 run passed 6/6 lifts with zero false stops
  over five ramp passes. See the
  [lift-stop results](../../testing/subsystem4/TEST_LOG.md#results). The
  video caption calls the lift demonstration collision detection; it is
  evidence of lift-stop, not an impact test.
- Still to build (both assessed): motion classification (#42) and turn rate
  from the magnetometer (#43).
- Minimum-bit (CR1): the packed output is a 4-byte `IMU_PACKED_MSG`
  (S4-U06); before/after size figures are still to record (S4-U14).

Tests and their results: `testing/subsystem4/TEST_LOG.md`. Plan: the Buddy 4
section of `testing/TEST_PLAN.md`.

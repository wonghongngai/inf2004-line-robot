# Buddy 4: IMU, humps, tilt (subsystem 4)

The IMU task samples an LSM303DLHC (accelerometer + magnetometer) every 20 ms,
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

## Status (13 Nov 2026)

- Detection thresholds in `subsystem4.h` (`S4_TILT_DELTA_MG`,
  `S4_HUMP_ENTRY_MG`, …) are **placeholders**: tune them from logged IMU data
  before treating them as final (S4-U08, issue #25).
- The hump height estimator keeps the climb height after the descent; the
  host test `s4/hump_height` expects 0. Decision pending (#5).
- Collision detection is being **removed** on the instructor's instructions
  and replaced by a lift-stop (#53).
- Still to build (both assessed): motion classification (#42) and turn rate
  from the magnetometer (#43).
- Minimum-bit (CR1): the packed output is a 4-byte `IMU_PACKED_MSG`
  (S4-U06); before/after size figures are still to record (S4-U14).

Tests and their results: `testing/subsystem4/TEST_LOG.md`. Plan: the Buddy 4
section of `testing/TEST_PLAN.md`.

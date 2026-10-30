# Offline combined driving build

Run `bash build_make/build_combined_current.sh` using MSYS bash on this machine.
Flash `build_make/mtk3pico_smp1_usb_cdc_all_test.uf2`.

This profile combines S2 motors/encoders, S3 TRACK line sensing, and S4 IMU.
S1 command/telemetry queues are initialized, but Wi-Fi and MQTT are disabled.
Distance and hump-height calculations use the existing encoder calibration.
Barcode decoding is disabled: GP7/GP27/GP0 are left/centre/right line inputs.

The default combined profile now follows the white area with GP27 centre
and GP0 right. GP7 remains sampled for diagnostics but does not affect steering.
Inputs remain 1=black, 0=white:

| Centre | Right | Action | Left/right PWM |
| --- | --- | --- | --- |
| White | White | Straight | 12.5 / 14.2 |
| White | Black | Right pivot | 20 / 0 |
| Black | White | Left pivot | 0 / 14.2 |
| Black | Black | Latched stop; reboot | 0 / 0 |

Side detections choose steering immediately. After either a left or right
turn, both centre and right must remain white continuously for 100 ms before
straight movement resumes. A black reading resets this white confirmation;
the opposite steering input overrides immediately. Both black still stops.
The previous 500 ms right-only hold is removed. Pause clears steering memory.
Sensor-pattern/action changes print as [WHITE STEER] logs, with sampled raw
GPIO and filtered transitions also printed as [IR RAW].

The previous corner-memory/search routine is bypassed in this profile.
Black-line LOST/SEARCH and junction classifications do not override these
centre/right rules. Queue errors, stale snapshots, sensor faults, manual
pause, IMU tilt/collision/faults still stop movement. Straight and outer-wheel
power use the existing acceleration ramp. No search deadline is enabled.
Both white can also mean the robot is off the intended track; this two-sensor
rule alone cannot distinguish that from the desired white area. Switch off
motor power to end a trial. Pivot travel is excluded from distance/height input.

Start with wheels raised and keep stationary for IMU calibration. Verify
physical wheel direction and sensor polarity (1 on black) before floor tests.
Host simulations and successful compilation do not validate physical tracking.

For networking, restore `config/wifi_secrets.h` and build with
`ROBOT_MODE=all_test WIFI=cyw43 NET=lwip MQTT=1 TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0`.

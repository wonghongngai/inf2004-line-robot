# all_test offline profile

Use this whole-robot test harness to try line following, encoder feedback,
IMU sensing and obstacle avoidance without Wi-Fi or MQTT. The
[integrated profile](../../README.md#1-which-firmware-to-use-integrated-or-all_test)
is the final firmware for system acceptance.

```sh
sh build.sh all_test_offline
```

Flash `build_make/robot-all_test-offline.uf2`. Keep the robot stationary for
IMU calibration. The offline profile starts automatically once at least
five seconds have elapsed, the IMU is ready and S5 releases its startup hold.

## Sensors and motion

- The default SIDE layout uses GP27 and GP0 for steering and the offset GP28
  sensor for barcodes. Digital inputs read 1 on black and 0 on white.
- Straight tracking uses both wheels and encoder trim. The current base
  PWM values are 15% left and 19% right in
  [subsystem2.h](../subsystem2/subsystem2.h).
- At a junction, a mapped barcode selects a move. The offline profile starts
  with an empty map and cannot load one over MQTT. It normally crosses
  straight, and an incomplete barcode can stop the harness.
- IMU faults, lift-stop and obstacle safety remain active with the default
  build flags. Tilt is reported and can suppress S5 detection.

For a trial that uses an MQTT letter map and dashboard, build
`sh build.sh all_test`. See the [root README](../../README.md#32-mqtt)
for broker and dashboard setup.

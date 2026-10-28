# Motion control with line sensing and IMU

Run commands from `build_make` in the configured WSL/Linux terminal. The default
is `ROBOT_MODE=integrated`. Every mode includes the real motor and encoder
drivers. Standalone images do not compile, link, or start S3/S4; they also skip
the LED and template subsystem. The kernel's USB/WiFi/I2C services remain.

| Build | Behaviour after boot | UF2 (default SMP/USB/WiFi) |
| --- | --- | --- |
| `make ROBOT_MODE=motor_test -j8` | After 5 s, timed wheel/pivot pulses, then stops. No calibration needed. Raise wheels. | `mtk3pico_smp1_usb_cdc_wifi_motor_test.uf2` |
| `make ROBOT_MODE=integrated_motor_test -j8` | Same raised-wheel pulses with S3 IR and S4 IMU tasks active; prints PWM and sensor diagnostics after each pulse. | `mtk3pico_smp1_usb_cdc_wifi_integrated_motor_test.uf2` |
| `make ROBOT_MODE=motion -j8` | Motors idle; accepts `subsystem2_send_command()` from application code. | `mtk3pico_smp1_usb_cdc_wifi_motion.uf2` |
| `make ROBOT_MODE=motion MOTION_DEMO=1 -j8` | After 5 s: forward 20 cm, left 90 degrees, forward 20 cm, right 90 degrees. 1.5 s pauses; stops after sequence or timeout. Needs motion calibration. | `mtk3pico_smp1_usb_cdc_wifi_motion_demo.uf2` |
| `make ROBOT_MODE=integrated -j8` | Waits for calibration/readiness, then follows the line and executes mapped junction turns. | `mtk3pico_smp1_usb_cdc_wifi_integrated.uf2` |
| `make ROBOT_MODE=integrated TEST=1 -j8` | S2/S3/S4 software self-tests; automatic driving disabled. | `mtk3pico_smp1_usb_cdc_wifi_integrated_test.uf2` |

There is no serial command parser: the USB console prints status, and the
standalone demo supplies commands automatically. `TEST=1` and the motor bench
profile cannot be combined. Give the options on every build. Integrated mode
requires `CONSOLE=usb_cdc CRASH_REPORT=0` because the right IR sensor uses GP0.

### Checking all three subsystems on hardware

```sh
make ROBOT_MODE=integrated_motor_test -j8
```

Flash the `_integrated_motor_test.uf2` image with wheels raised. This is a
combined hardware diagnostic; it uses timed motor pulses, keeps IR decisions
paused, and does not run autonomous navigation or IMU safety stops. Normal
`integrated` mode uses calibrated motion plus those safety stops.

Expected evidence:

- `[MOTOR] PWM reset_done=1 counters L=1 R=1 check=0`: the PWM peripheral is
  live, both counters advance, and motor pins select PWM. This is checked
  before any nonzero power is requested; failure prevents task startup.
- `[APP] INTEGRATED MOTOR TEST`: confirms S2, S3 and S4 were selected.
- `[S2 HW] PWM check=0 counting=1/1 compare=400/0` for the left-only pulse;
  `0/400` for right-only and `400/400` for both wheels at 40%.
- `S3 IR heartbeat=0`: the sensor task is providing fresh snapshots. State 4
  is deliberately paused. The bits are digital snapshots, not calibrated
  black/white labels in this diagnostic.
- `S4 IMU ... sensor_ok=1 calibrated=1` with increasing sample counts and
  no read errors: the physical IMU is connected and sampling.
- Actual wheel movement and changing encoder ticks: must be observed on the
  robot; register checks cannot prove motor supply or wiring.

The earlier nonmoving-wheel bug was in PWM initialization: kernel startup
releases PWM reset only when `USE_PTMR=1`, but this project uses `USE_PTMR=0`.
`pwm_set_pin()` now explicitly deasserts PWM reset and waits for RESET_DONE,
without resetting other slices. The old boot banner said "all subsystems"
even in standalone builds; each mode now prints its actual subsystem set.
Motor pin assignments agree with [Cytron's Robo Pico example](https://github.com/CytronTechnologies/Cytron-ROBO-PICO/blob/main/Getting_Started_Guide/CircuitPython/DC_Motor/code.py).
The motor driver now explicitly sets about 10 kHz PWM (divider 12.5 at the
port's 125 MHz system clock). The old reset-default divider would have produced
about 125 kHz with TOP=1000, exceeding the 20 kHz maximum in the
[Robo Pico datasheet](https://www.farnell.com/datasheets/4248509.pdf). Diagnostics
verify divider/TOP readback as well as counter activity and pin selection.

## Wiring

GPIO numbers, not header pin numbers:

| Function | GPIO |
| --- | --- |
| Left motor forward / reverse | GP8 / GP9 |
| Right motor forward / reverse | GP10 / GP11 |
| Left encoder A / B | GP26 / GP6 |
| Right encoder A / B | GP2 / GP3 |
| IR left / centre / right | GP7 / GP27 / GP0 |
| LSM303DLHC I2C0 SDA / SCL | GP4 / GP5 |

Use the existing Robo Pico motor terminals/power supply and a common ground.
Verify M1 is the left wheel, M2 is the right wheel, and both turn forward during
the raised-wheel test. Connect the IMU to I2C0 (`iica`), with pull-ups to 3.3 V.
The unused I2C1 boot assignment is GP14/GP15; GP6/GP7 previously conflicted with
the encoder/IR inputs. Keep these unused pins free of other circuitry.

## Calibration and first drive

Calibration converts encoder ticks into physical distance and angle. It is
required for distance commands, turns, and integrated driving; the timed motor
bench test bypasses those calculations so wiring can be checked first.

1. Flash `motor_test`, raise the wheels, and verify direction, sides, and
   changing encoder counts as described in [HARDWARE_TEST.md](HARDWARE_TEST.md).
2. Flash plain `motion` with calibration still disabled. The motors stay off,
   but the `[MOTION]` serial line reports raw ticks every second. Manually turn
   each wheel through several measured revolutions. Divide absolute tick change
   by wheel revolutions times measured wheel circumference in centimetres.
   Enter the two values in `S2_LEFT_TICKS_PER_CM` / `S2_RIGHT_TICKS_PER_CM` in
   `subsystem2.h`. Counts use all quadrature transitions observed by this driver;
   do not substitute a datasheet pulse count without checking its definition.
3. With motors unpowered, hold the left wheel fixed and pivot the chassis left
   through a measured angle, recording right-wheel ticks. Divide by degrees for
   `S2_RIGHT_TURN_TICKS_PER_DEGREE`. Repeat right with the right wheel fixed for
   `S2_LEFT_TURN_TICKS_PER_DEGREE`. These are one-wheel pivots, not counter-rotation.
4. Enter practical PWM settings, set `S2_CALIBRATION_READY=1`, and use
   `motion MOTION_DEMO=1` on a clear surface to validate distance and angle.
   Adjust for tyre slip and tune PID gains. Zero PID gains give no straight-line
   correction. Each demo stage has a 15 s timeout; encoder stalls stop sooner.
5. Measure all three IR digital levels on black and on white. In
   `robot_control.h`, set `S2_LINE_BLACK_RAW` / `S2_LINE_WHITE_RAW` (left bit2,
   centre bit1, right bit0), then `S2_LINE_CALIBRATION_READY=1`. Each bit must
   change between surfaces. Select SIDE layout for the existing offset barcode
   sensor arrangement; TRACK layout disables barcode recognition.
6. Flash `integrated`. Keep the robot stationary for IMU baseline calibration
   (50 samples, roughly 1 s). Driving starts after at least 5 s and only when
   motion calibration, line calibration, and a healthy calibrated IMU are ready.
   Expect `[ROBOT] armed` followed by `[MOTION]` state/tick/PWM reports.

Polling uses a requested 1 ms sleep and a 1 ms kernel tick. Kernel rounding adds
one tick, so actual polling is about 2 ms minimum plus scheduling/console work.
Check counts at the intended driving speed against slow/manual measurements.
If edges are missed, reduce speed or implement a shared GPIO IRQ/PIO encoder
capture before relying on distance accuracy. This implementation does not
claim lossless encoder capture.

## Integration behaviour

`robot_control.c` runs inside the existing motor-owning task. It calls S3's
`line_service_receive()` / `snapshot()` and S4's `subsystem4_get_latest()`;
it does not read their GPIO or I2C directly. S3/S4 processing code is unchanged.
The only S3 edit allows its demo-enable macro to be overridden by the build,
so there is exactly one navigation-event consumer.

Line position supplies left/right steering while the existing encoder control
checks progress. Default mapping: A = left 90 degrees, B = right 90 degrees,
C = straight, Z = left 180 degrees. S3 stores a barcode letter and issues its
mapped command at the next junction. Decisions pause during an encoder-based
turn and resume after completion, allowing 30 ms for fresh line samples. SIDE
layout must match S3's documented sensor placement and barcode/track geometry;
the controller does not add a junction-centering advance. Tune physical sensor
placement and pivot calibration on the real track before expecting clean turns.

The motor-owning task also supplies average encoder travel to
`subsystem4_add_distance_mm()` during forward travel, excluding pivot turns
and stationary states. Distance conversion retains fractional millimetres.
S4 accumulates fractional height before rounding and retains the highest
completed hump estimate. Distance requires positive measured left/right
ticks-per-centimetre values; motor PWM cannot supply a real distance scale.
S4 reports height unavailable when a hump has no wheel-distance input.

Line loss/search, an unmapped junction, queue/timing faults, stale line data,
IMU errors/staleness, collision, tilt, a turn timeout, or a motion fault stop
motors and latch the integrated stop until reboot. Humps alone allow travel to
continue; speed compensation requires real terrain measurements. IMU freshness
is inferred from sample-count changes with a 100 ms deadline. IMU events are
snapshots; pulses shorter than the adapter's scheduling interval are not queued.

Plain standalone motion accepts the existing forward/left/right/reset commands
through `subsystem2_send_command()` and exposes `subsystem2_get_status()`.
Check each return code and wait for IDLE before another move; a successful send
means queued, not completed. Normal requests use a bounded latest-command queue.
STOP uses a separate event flag so a full movement queue cannot reject it, and
discards pending movement requests. In integrated/demo mode STOP latches until
reboot; other external commands are rejected because the automatic controller
owns movement. Bench mode rejects the command API; use motor power to stop a
bench pulse immediately.

## Verification

From the repository root:

```sh
sh testing/integration/run_robot_host_tests.sh
PICO_SDK_PATH=/path/to/pico-sdk sh testing/integration/build_robot_profiles.sh
```

Host tests execute production adapter/controller code with simulated S3/S4
interfaces. They check readiness, steering, all four mapped junction commands,
resuming after turns, latched sensor stops, encoder stalls, standalone distance/
angle completion and invalid commands. Builds cover integrated, integrated tests,
single-core integrated, standalone motion, standalone demo and motor bench.
The register regression also executes the real BSP PWM and motor driver
against a reset-held peripheral model, checking reset release, advancing
counters, PWM outputs, stopped clocks, pinmux faults and reset timeout.
The build script includes the combined hardware diagnostic (seven profiles).
These checks do not establish physical wheel movement or track performance.

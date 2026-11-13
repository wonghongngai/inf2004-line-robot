# Test plan

What each buddy tests, in what order, and what "done" means.
Written 2026-11-13 from the code and test logs on `chore/fix-host-tests`.

- **This file is the plan.** Each `subsystemN/TEST_LOG.md` holds the test
  cases and results (the evidence). `STATUS.md` is the summary.
- When you start a test listed here, copy its row into your `TEST_LOG.md`
  *Test cases* table first (README step 1). Then run it, save the log, and add
  a *Results* row.
- **Proposed IDs** carry on from the highest ID already used, so they never
  clash. Rename them if you need to; just don't reuse an old ID.
- **`[set]`** marks a pass threshold the team still has to agree on. Agree it
  *before* running the test, and write the number into the test case.

## Test levels

Every feature climbs these levels in order. Don't tune on the track (L5)
something that hasn't passed on the bench (L3).

| Level | What | How | Who sees the result |
|---|---|---|---|
| **L1 host** | Logic on the PC, no Pico | `sh testing/run_host_tests.sh <area>/` | anyone, in seconds |
| **L2 on-target** | `TEST_CHECK`s on the Pico | `make TEST=1`, flash `*_test.uf2`, save the serial log | `[TEST] <ID> PASS/FAIL` lines |
| **L3 bench** | Real hardware, robot not driving (on a stand or held) | manual steps + serial log / scope / video | the tester |
| **L4 integrated** | Feature inside `ROBOT_MODE=integrated` | flash the integrated image | the team |
| **L5 track** | Tuned and repeatable on the real course | N repeated runs, video | the team / the demo |

---

## Step 0: Everyone, before writing new tests (about 1 hour each)

1. **Fix the ID drift in the TEST_LOGs.** The on-target code already has real
   checks, but the *Test cases* tables still describe the course template's
   LED/sample tests under the same IDs:

   | Area | In the code (`subsystemN_test.c`) | In `TEST_LOG.md` |
   |---|---|---|
   | S1 | S1-U01–U10: packet sizes, ranges, queues, barcode map | S1-U01–U03 = template LED tests |
   | S2 | S2-U01–U04: message size, forward target, STOP, stall → FAULT | S2-U01–U02 = template sample tests |
   | S4 | S4-U01–U07: snapshot, sensor, calibration, sample rate, 20 ms cycle, 4-byte packing, ramp range | S4-U01–U02 = template tests |

   Rewrite those rows to describe what the code checks. The template tests
   were never run, so nothing is lost. Do the same for INT-01/INT-02, which
   are template examples: mark them *Retired*.
2. **Run what already exists** (the "Have" tables below) and record the
   results. Most of these tests have never had a Results row.

---

## Buddy 1: Wi-Fi, MQTT, telemetry (S1)

**Have (run now):**

| Test | Level | How | Status |
|---|---|---|---|
| `s1/protocol`: 80-bit telemetry codec | L1 | `sh testing/run_host_tests.sh s1/protocol` | PASS |
| `s1/binary_cli`: `tools/mqtt_binary.py` encoder/decoder | L1 | `sh testing/run_host_tests.sh s1/binary_cli` | PASS |
| S1-U01–U10: packet sizes, range rejection, queues, barcode map | L2 | `make TEST=1` | never run on the Pico |
| `s1/barcode_queue` | L1 | — | **blocked**: the code it tests isn't committed (zhi xian) |

**New:**

| ID | What | Level | How | Pass when |
|---|---|---|---|---|
| S1-U11 | Wi-Fi joins after boot | L3 | Power on 5×, read the connect time from the serial log | connects 5/5 within `[set]` s |
| S1-U12 | Telemetry publish rate and validity | L3 | Broker from `tools/mosquitto-local.conf`; subscribe and decode with `tools/mqtt_binary.py` for 5 min | ~1 packet/s, every packet is 10 bytes and decodes valid, no gaps over `[set]` s |
| S1-U13 | Reconnect after broker restart and after AP off/on | L3 | Stop the broker for 30 s, then restart it; repeat with the hotspot | publishing resumes within `[set]` s; motors unaffected (Wi-Fi loss must not stop driving) |
| S1-U14 | PAUSE / RESUME / STATUS from the laptop | L3→L4 | Send each command 10×, film the wheels with the console visible | 10/10 obey; PAUSE stops within `[set]` ms |
| S1-U15 | Letter-to-movement map sent from the PC over MQTT: valid, duplicate letters, reserved bits set | L3 | Publish 3 maps to `<id>/mapping` from the PC, then drive over a mapped barcode and junction | valid map **applied** (robot does the mapped move), both invalid rejected, old map kept (#38; pairs with S3-U27) |
| S1-U16 | Barcode result events (`<id>/barcode`) | L4 | After the code is pushed: 20 barcodes on the track | 20/20 events arrive, frame IDs in order |
| S1-U17 | Command flood does not starve control | L2/L3 | Send 100 commands in 1 s | control loop shows no late cycles; extra commands rejected and counted, never blocking |

**Done when:** U01–U17 pass, and telemetry runs through a full system run (SYS-05).

---

## Buddy 2: Motors, encoders, PID, Vehicle Controller (S2)

**Have (run now):**

| Test | Level | How | Status |
|---|---|---|---|
| `s2/motion_protocol`: compact command/status bytes | L1 | `sh testing/run_host_tests.sh s2/` | PASS |
| `s2/motor_pwm`: PWM driver on modelled registers | L1 | same | PASS |
| `int/robot_control`: VC line/barcode/IMU logic, line-stop re-arm | L1 | `sh testing/run_host_tests.sh int/robot_control` | PASS (209 checks) |
| S2-U01–U04: message size, forward target, STOP, stall → FAULT | L2 | `make TEST=1` | never logged |

**Decided 13 Nov (#4):** `integrated` is the final firmware. Port the
all_test tuning into it (#8). The `all_buddies` host tests were retired (#9);
their line-stop re-arm checks now live in `int/robot_control`. Every L4 test runs on the
`ROBOT_MODE=integrated` image.

**New:**

| ID | What | Level | How | Pass when |
|---|---|---|---|---|
| S2-U05 | Motor direction and sides | L3 | `motor_test`, wheels raised, follow `HARDWARE_TEST.md` | M1 = left, M2 = right, both forward |
| S2-U06 | Encoder ticks per cm, each wheel | L3 | Push or drive 100 cm × 3, read the counts | spread between runs ≤ 2 %; value written into `CALIBRATION.md` |
| S2-U07 | Ticks per degree (pivot) | L3 | 4 × 90° and 1 × 360° pivots, check against floor marks | within `[set]` ° per 360° |
| S2-U08 | Drives straight | L4 | 2 m with no line; measure the sideways drift | ≤ `[set]` cm |
| S2-U09 | Barcode turn angles | L4 | 10 × each of left 90°, right 90°, U-turn 180° | within ±`[set]` °, and the line is reacquired after each |
| S2-U10 | Speed PID step response (gains are currently 0) | L3 | Step the target speed, log the encoder speed | settles within `[set]` ms, overshoot ≤ `[set]` % |
| S2-U11 | STOP latency | L3 | STOP command → wheels stopped, timed with a GPIO toggle + scope or slow-motion video | ≤ `[set]` ms |
| S2-U12 | Control loop timing | L4 | 5 min integrated run; read the late-control count that `motion_task` prints | 0 late cycles; max work time recorded |
| S2-U13 | Reverse by distance (`moveBackward`, #39) | L3 | Reverse 10 / 20 / 30 cm, 3× each, measure on the floor | within `[set]` cm; stops cleanly; works for both encoder polarities |
| INT-13 | Host test of the S5 avoidance hand-over | L1 | New `int/robot_avoidance`: `robot_control.c` + the real `s5_executor.c`, kernel flags stubbed | VC hands control to S5, takes it back, and safe-stops on failure |

**Done when:** U05–U12 pass with calibration recorded, and the VC passes
INT-08–INT-10 in integrated mode.

---

## Buddy 3: IR line following + Code 39 barcode (S3)

**Have (run now):**

| Test | Level | How | Status |
|---|---|---|---|
| `s3/line_barcode`: S3-U02–U13, U18–U22, U25–U26 | L1 | `sh testing/run_host_tests.sh s3/` | PASS |
| `s3/pptx_samples`, `s3/count_a`: slide barcode fixtures | L1 | same | PASS |
| `int/line_service`: 1 ms sampling, pins, 30 ms freshness | L1 | `sh testing/run_host_tests.sh int/line_service` | PASS |
| S3-U01, U14–U17, U23–U24, U27 | L2 | `make TEST=1` | PENDING since 25–27 Oct |

> ⚠️ **Layout:** both builds now default to SIDE layout: GP28 is the offset
> barcode scanner and GP27/GP0 steer (`ALL_TEST_LINE_LAYOUT` and
> `S2_LINE_LAYOUT`). The top-level README still describes the older TRACK
> layout (GP7/GP27/GP0, barcode off). Check that the sensors are physically
> mounted to match SIDE before any S3 hardware test, and fix the README.

**New:**

| ID | What | Level | How | Pass when |
|---|---|---|---|---|
| S3-U28 | IR calibration on the real track | L3 | Log the raw value of each sensor over black tape and over white floor, under the room lights | clear gap between the black and white values on every sensor; values in `README.md` |
| S3-U29 | Follows a straight 18 mm line | L4 | 3 m straight, 5 runs, cruise speed | 5/5 stay on the line |
| S3-U30 | Follows the course's curves and corners | L4 | Each curve or corner type × 5 | 5/5 without losing the line |
| S3-U31 | Line lost → search → reacquire on hardware | L4 | Lift the robot off the line and put it back; time it | reacquires within `[set]` ms; reports FAULT after `LINE_BARCODE_SEARCH_MS` if the line isn't found |
| S3-U32 | The 4 letters received over MQTT (#38), at driving speed | L4 | Send a map from the PC; 10 passes × 4 barcodes, both directions, each followed by a cross-junction | ≥ `[set]` / 10 correct letters each, and the mapped move done at the next junction; log the decode rate per letter |
| S3-U35 | All A–Z decode in both directions | L3 | Print A–Z from the tec-it Code 39 generator; pass each barcode both ways at cruise speed | 26/26 letters correct in both directions (prof requirement, 13 Nov) |
| S3-U33 | Bad barcodes cause no action | L4 | An unmapped letter and a damaged barcode, 5 passes each | 0 wrong commands |
| S3-U34 | Speed limit for decoding | L3/L4 | Increase the speed until decoding fails | highest reliable speed recorded; cruise speed set below it |

**Done when:** U01–U34 pass, and barcode turns work end-to-end (INT-08).

---

## Buddy 4: IMU, humps, tilt, lift-stop (S4)

**Have (run now):**

| Test | Level | How | Status |
|---|---|---|---|
| S4-U01–U07: snapshot, sensor, calibration, sample rate, 20 ms cycle, packing, range | L2 | `make TEST=1` | never logged |
| `s4/hump_height` | L1 | `sh testing/run_host_tests.sh s4/` | **FAIL**: the test expects the height to fall back to 0; since `7f823b8` the code keeps the climb height. Decide which is right |

**New:** the thresholds in `subsystem4.h` are placeholders, so the first step
is to measure real data.

| ID | What | Level | How | Pass when |
|---|---|---|---|---|
| S4-U08 | Noise at rest and while driving on flat floor | L3 | Log the IMU for 60 s still and 60 s driving (save the CSV in `logs/`) | noise recorded; thresholds set clearly above it |
| S4-U09 | Hump detected | L4 | 10 passes over each real hump, 10 runs on flat floor | 10/10 detected, 0 false detections |
| S4-U10 | Highest hump height accuracy | L4 | Measure each hump with a ruler; 5 passes each | within ±`[set]` mm; with several humps, the highest is reported |
| S4-U11 | Tilt detection | L3 | Tilt the robot by hand past the limit, 10× | 10/10 flagged within `[set]` ms |
| S4-U12 | Lift stops the motors (replaces collision detection, removed 13 Nov; #53) | L4 | Lift the robot off the floor while driving, 10×; drive over each hump 10× | 10/10 stop and stay stopped; 0 false stops over humps |
| S4-U13 | Replay recorded IMU logs on the PC | L1 | New `s4/imu_replay` host test: feed the U08–U12 logs into `task_imu.c` | same results as on the robot; tuning changes can then be checked offline |
| S4-U14 | CR1 minimum-bit change | L2 | `arm-none-eabi-size` and `.map` before and after | numbers recorded (the README says this is still the PRE version) |

**Done when:** U01–U14 pass, and the tilt/obstacle timing check (INT-11) passes.

---

## Buddy 5: Ultrasonic scanning, profiling, avoidance (S5)

**Have (run now):** S5 has the most tests, but few have Results rows.

| Test | Level | How | Status |
|---|---|---|---|
| S5-U09, U21–U22, U28, U31–U38: packing, servo maths, fine-scan window, profile, plan, bypass table | L2 | `make TEST=1` | in the code; no Results rows |
| S5-U01, U04, U05, U16 | L3 | manual | PASS (logs not saved) |
| S5-U02: accuracy ±2 cm over 5–100 cm | L3 | manual | **FAIL before the hardware-timer fix: re-run** |
| S5-U06: detection → action ≤ 2 s | L3 | manual | **FAIL (2299 ms) before the `FINE_BUDGET_POINTS` fix: re-run** |
| S5-U03, U07, U08, U10–U13, U15 | — | planned | not written yet |

**New:**

| ID | What | Level | How | Pass when |
|---|---|---|---|---|
| S5-U39 | Obstacle logic on the PC | L1 | Move the pure `obstacle_logic.c` checks (filters, profile, planner) into a host test `s5/obstacle_logic`, with kernel types stubbed like `robot_host_stubs.h` | same checks as on target; runs in seconds |
| S5-U40 | Detect at ≥ 30 cm and stop ≥ 10 cm from the obstacle (NFR5-1) | L4 | 10 approaches to a box at cruise speed | 10/10 detected at ≥ 30 cm, stopped ≥ 10 cm away |
| S5-U41 | Avoids a box | L4 | Box centred, offset left and offset right, 5 runs each | 15/15 pass without touching it |
| S5-U42 | Reacquires the line within 50 cm (NFR5-4) | L4 | Same runs as U41; mark where the line is reacquired | 15/15 within 50 cm |
| S5-U43 | Ignores invalid readings (NFR5-5) | L3 | Point at open space, a soft target, and very close (< 2 cm) | never the only basis of a decision; logged as invalid |
| S5-U44 | Hump is not treated as an obstacle (A5-2) | L4 | 10 hump passes with S5 active | 0 false obstacle stops (pairs with INT-11) |

**Done when:** U01–U44 pass, and avoidance works in integrated mode (INT-10).

---

## Integration (`testing/integration/`)

| ID | What | Level | Pass when |
|---|---|---|---|
| INT-03 | VC logic with simulated S3/S4 | L1 | PASS 2026-11-13 (`af2da1c`) |
| INT-04/06 | All firmware profiles build | build | `sh testing/integration/build_robot_profiles.sh` passes: rerun on every PR that touches `app_program/` |
| INT-07 | Telemetry and commands, S1 ↔ VC (IF-05, IF-06) on the robot | L4 | S1-U12 + S1-U14 pass in the integrated build |
| INT-08 | Barcode command at a junction gives the right turn (IF-07) | L4 | 4 commands × 5, 20/20 correct |
| INT-09 | IMU safety stop in integrated mode (IF-08) | L4 | lift / tilt → stops 10/10 (collision detection removed 13 Nov) |
| INT-10 | S5 takes control, avoids, hands back (IF-10, IF-11) | L4 | S5-U41 passes in the integrated build |
| INT-11 | Hump vs obstacle timing (open risk) | L4 | tilt flag is set before the ultrasonic reacts; 0 false obstacle stops on 10 hump passes |
| INT-12 | Host suite clean | L1 | `sh testing/run_host_tests.sh` all PASS before each merge to `main` |
| INT-13 | VC ↔ S5 hand-over on the PC | L1 | see Buddy 2 |

## System (`testing/system/`): the whole robot against the brief

| ID | What | Pass when |
|---|---|---|
| SYS-01 | Full course, start to finish | 3 runs in a row with no touches |
| SYS-02 | Every barcode on the course is obeyed | all correct in SYS-01 runs |
| SYS-03 | Highest hump reported over MQTT matches the measured height | within S4-U10's tolerance |
| SYS-04 | Obstacle avoided and line reacquired during a full run | passes in the SYS-01 runs |
| SYS-05 | Telemetry continuous for a whole run | no gap over `[set]` s; final state = finished |
| SYS-06 | Demo video | one uncut run with the console/MQTT view, linked in `videos/README.md` |

---

## Order of work

1. **Step 0** (everyone, in parallel): fix the TEST_LOG IDs, then run and log
   all L1 + L2 tests that exist.
2. **Unblock:** zhi xian pushes the missing S1/S2 code; Buddy 4 decides the
   hump test; the team decides `integrated` vs `all_test` and the barcode mounting.
3. **Bench (L3), in parallel:** S1-U11–U15, S2-U05–U07/U10–U11, S3-U28/U34,
   S4-U08/U11, S5-U02/U06/U43.
4. **Integrated (L4):** INT-07–INT-11, then each buddy's L4 tests.
5. **Track (L5) and SYS:** only after L4 passes, then SYS-01–SYS-06.

At each weekly check-in, move `STATUS.md` cells only for tests that have a
Results row with a commit hash and a saved log.

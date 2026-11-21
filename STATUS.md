# Project status

**This is the one page that shows what is done and what is left.**
What to test next, and how, is in [testing/TEST_PLAN.md](testing/TEST_PLAN.md).
Where the code differs from the Week 6 / PreTri plan, and why, is in [DEVIATIONS.md](DEVIATIONS.md).
Team decisions and their reasons are in [DECISIONS.md](DECISIONS.md) (latest: the M1 decisions, 20 Nov: #5, #6, #48, #54, #64–#68).
Last updated: 2026-11-21, event-bit table checked against the code (#50, INT-14) · 2026-11-20, M1 decisions (`DECISIONS.md`; tilt report-only, hump test fixed) · subsystems 1, 2, 4 and 5 test results (`chore/m0-housekeeping` from `65ba452`; #14, #17, #24, #28) · subsystem 3 rows 2026-11-19 (`cc239d5`) · other sections as of 2026-11-13 (`double11` @ `1d78677`)

## Rules

1. **A cell is ✅ only when it has evidence**: a row in the right
   `testing/**/TEST_LOG.md` with a commit hash and a saved log, photo or
   video. Put the link in the **Evidence** column. "It worked when I tried
   it" counts as 🟡, not ✅.
2. **"Integrated" means `ROBOT_MODE=integrated`.** Working in `all_test`,
   `line_follow_test` or `barcode_test` alone does **not** count.
   **Decided 13 Nov (#4):** `integrated` is the final firmware for the
   demo. `all_test` is a test harness only: a behaviour change is not done
   until it is in the integrated build and tested there.
3. Every row that isn't ✅ has a **Next action** that one person can pick up.
   When there's a GitHub issue for it, link the issue in that column.
4. Update this file in the same PR or commit as the test-log row that
   changes it.

**Legend:** ✅ done with evidence · 🟡 partly done, or done without evidence ·
⬜ not started / not run · ❌ tested and failed · — not applicable

**Stages:**
- **Code**: written and compiles.
- **Host/Unit**: unit tests pass, on the PC (host) or on the Pico (`make TEST=1`).
- **Bench HW**: works on real hardware with the robot not driving (on a stand or held).
- **Integrated**: works in `ROBOT_MODE=integrated` firmware.
- **Track tuned**: tuned and repeatable on the real track.

---

## Subsystem 1: Wi-Fi / MQTT / telemetry (Buddy 1)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| 15-byte telemetry packing (CR1) | ✅ | ✅ host `s1/protocol`; Pico S1-U01–U04 (U01 fixed and re-run 20 Nov) | ⬜ | ⬜ | — | `testing/subsystem1/logs/2026-11-20_host.txt`, `testing/subsystem4/logs/2026-11-20_target_test.txt` | S1-U12: decode real packets on the PC (#15) |
| Command and map queues (STATUS, REVERSE, reserved bits, 3-byte barcode map) | ✅ | ✅ S1-U05–U10, U18 (Pico) | ✅ 21 Nov | ✅ 21 Nov | — | `testing/subsystem3/logs/2026-11-19_target_test.txt`; robot: `testing/integration/logs/2026-11-21_2129_robot_mqtt_test.txt` (S1-U14), `…_1944/1957/2001_robot_mqtt_test.txt` + `…_s1u15_drive_robot.txt` (S1-U15) | Done: S1-U14 PASS, S1-U15 PASS on the robot (#15, #38) |
| MQTT connect + reconnect after AP or broker loss | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | — | Define S1-U tests (connect, publish rate, reconnect time); run on the bench with a broker and save logs |
| Telemetry publish (state, speed, hump, obstacle…) | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | — | Check every field decodes correctly on the subscriber side; record the publish rate |
| Barcode result events (`<id>/barcode`, QoS 1) | 🟡 | ❌ `s1/barcode_queue` does not compile: code not pushed (#3) | ⬜ | ⬜ | ⬜ | `testing/subsystem1/logs/2026-11-20_host.txt`, `testing/subsystem1/BARCODE_MQTT.md` | Run `barcode_queue_test.c` and log the result; then do a bench test with a real broker |
| Letter-to-movement map received over MQTT and **applied** | ✅ | ✅ INT-03 (integrated, simulated) | ⬜ | ⬜ | ⬜ | No built-in map in either build; straight until a map arrives; telemetry bit 86 + dashboard "Letter map" (21 Nov, `testing/integration/logs/2026-11-21_map_flag_host.txt`) | On the robot: S1-U15 and S3-U27. No A–Z print test (decided 21 Nov, `DECISIONS.md`): use large prints |

## Subsystem 2: Motors, encoders, PID, Vehicle Controller (Buddy 2)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| Motor PWM driver | ✅ | ✅ INT-05 | 🟡 | ⬜ | — | `testing/integration/logs/2026-10-28_pwm_fix.txt`, `testing/subsystem2/logs/2026-11-20_host.txt` | Flash `motor_test` with the wheels raised, follow `HARDWARE_TEST.md`, log the result |
| Encoders + odometry | ✅ | ⬜ | 🟡 | ⬜ | ⬜ | `CALIBRATION.md`: old calibration not validated for IRQ counts | Recalibrate counts per cm and per degree; log them |
| Speed PID | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | `CALIBRATION.md`: gains still 0 | Tune the gains on the robot; record step response / logs |
| Straight driving + turning | ✅ | ✅ INT-03 (simulated, 837 checks) | 🟡 in all_test only | ⬜ | ⬜ | Commits `ab62be5`, `1d78677`; port: `testing/integration/logs/2026-11-21_alltest_port_host.txt` | Ported 21 Nov (#8, row below). Flash `integrated` and retest on the track |
| Rest of all_test ported to integrated (#8): encoder straight trim; barcode-scan slow-down (×0.9, left inner 8 %); inner wheel back to cruise when a correction ends; junction advance straight from following (no stop) for all_test's 94 ticks (0.8 cm), pivots at 12.5 / 15 % with the S2 turn calibration and no tolerance; counter-rotating U-turn (−12.5 / 15 %) that ends on half the arc **and** centre black; S5 legs heading-held at 12.5 / 15 %, pivots at 20 % with S5's pivot radius, 150 ms hold settle, 20 s move timeout, junction grace after hand-back; interrupt-encoder ticks/cm (117.8 / 118.3) for speed, distance, S4 hump input and S5 legs; `[BARCODE FRAME/WIDTH/REJECT]` and IMU-stop logs. One set of constants in `subsystem2.h` for both builds | ✅ | ✅ INT-03 (simulated, 837 checks; 12 mutations caught) | ⬜ | ⬜ | ⬜ | `testing/integration/logs/2026-11-21_alltest_port_host.txt` | Track run in `integrated`: junctions (all four commands), an obstacle, hump height. Not ported on purpose: the incomplete-barcode stop (conflicts with #72), and test-harness-only modes (continue-on-sensor, run limit, TRACK layout corner search, hand-push encoder calibration) |
| Integrated line following uses all_test's motor numbers: 12.5/15 % cruise, +4 outer, inner 5 % then 0 % after 150 ms, 80 %/s ramp, no floor and no stall stop while following; all_test reads the same `S2_FOLLOW_*` constants (#8) | ✅ | ✅ INT-03 (simulated, 727 checks) | ⬜ | ⬜ | ⬜ | `testing/integration/logs/2026-11-21_motor_tuning_host.txt` | On the track: compare with all_test; the 21 Nov 17:41 run (20 % cruise) followed ~2.5 m, then a right-wheel stall fault in a hard correction |
| Integrated line model = all_test's straddle steering (C/R either side of the line), junctions from C/R 1/1 with a 5 cm advance before pivots (#8) | ✅ | ✅ INT-03 (simulated, 725 checks) | ⬜ | ⬜ | ⬜ | `testing/integration/logs/2026-11-21_straddle_host.txt` | On the track: follows the line, turns at a mapped junction. The rest of all_test is now ported (row above); the advance is all_test's real 0.8 cm, not 5 cm |
| Compact motion protocol v1 (CR1) | ✅ | ✅ `s2/motion_protocol` (host) | ⬜ | ⬜ | — | `testing/subsystem2/logs/2026-11-20_host.txt` | Move the other subsystems onto `subsystem2_send_command` |
| Motion control: distance target, STOP, encoder stall → FAULT | ✅ | ✅ S2-U01–U04 (Pico, simulated encoders) | ⬜ | ⬜ | ⬜ | `testing/subsystem3/logs/2026-11-19_target_test.txt` | Calibrate on the robot (S2-U06, U07), then STOP latency (S2-U11) |
| Vehicle Controller mode arbitration (safety > avoidance > line) | 🟡 | ✅ INT-03 (simulated) | ⬜ | ⬜ | ⬜ | `testing/subsystem2/logs/2026-11-20_host.txt` (209 checks) | Bench test with real S3/S4/S5 inputs |
| Integrated MQTT link: telemetry, PAUSE/RESUME/STATUS, letter map (#72) | ✅ | ✅ INT-03 (simulated, 346 checks) | ✅ 21 Nov | ✅ 21 Nov | — | `testing/integration/logs/2026-11-21_integrated_mqtt_host.txt`; robot: `…_1944_robot_mqtt_test.txt` (S1-U12), `…_2129_robot_mqtt_test.txt` (S1-U14), `…_s1u15_drive_robot.txt` (S1-U15) | Done on the robot: INT-07 PASS (S1-U12 + S1-U14) and S1-U15 PASS (#72). The pivot after a mapped junction stopped ~2° short on a stall fault (#8) |
| Ready state + MQTT START (#48) | ✅ | ✅ INT-03 (simulated, 228 checks) | ✅ 21 Nov | ✅ 21 Nov | — | `testing/integration/logs/2026-11-21_start_command_host.txt`, `testing/integration/logs/2026-11-21_ready_start_robot.txt` | Done (#48) |

## Subsystem 3: IR line following + barcode (Buddy 3)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| Line position from 3 IR sensors, noise rejection | ✅ | ✅ S3-U02–U03 (host + Pico) | ⬜ | ⬜ | ⬜ | `testing/subsystem3/logs/2026-11-19_host_s3_int.txt`, `2026-11-19_target_test.txt` | Run S3-U28: each sensor's comparator reads black/white on the real track (#22) |
| Line service API: calibration, pause/resume, sampling heartbeat | ✅ | ✅ S3-U14–U17 (host + Pico) | ⬜ | ⬜ | — | same logs | Bench check comes with S3-U28 (#22) |
| Line loss → search → reacquire / fault | ✅ | ✅ S3-U04–U06 (host + Pico) | ⬜ | ⬜ | ⬜ | same logs | S3-U31 on the track: lift the robot off the line, time reacquiring (#23) |
| Service faults latch until reset (event-queue overflow, sampling gap > 20 ms) | ✅ | ✅ S3-U11, U43–U44 (U43–U44 host only) | ⬜ | ⬜ | — | `testing/subsystem3/logs/2026-11-19_host_s3_int.txt` | Controller stops on a latched fault: integration part of S3-M06 |
| Code 39 decode A–Z, forward + backward | ✅ | ✅ S3-U07–U13, U36–U42 (host + Pico) | 🟡 only A and Z tested (commit `208188b`, no log) | ⬜ | 🟡 A only: large and medium print, both directions, in `all_test` (not integrated) | `testing/subsystem3/TEST_LOG.md` 19 and 20 Nov hardware rows, `logs/2026-11-19_barcode_hw.txt`, `logs/2026-11-20_barcode_hw.txt` (A forward 3/3 after the GP28 move) | S3-U35: all A–Z both ways (prof, 13 Nov), then the four command letters at driving speed (#23) |
| Junction detection + barcode → command at next junction | ✅ | ✅ S3-U25–U26 (host + Pico), U45–U46 (host) | ⬜ | ⬜ | 🟡 A → LEFT 29/29 in `all_test` (not integrated) | same | Track test at each junction type in integrated firmware; INT-08 (#33) |
| Run-time letter mapping (S3-U27) | ✅ | ✅ S3-U23–U24, U27 (host + Pico) | ✅ 21 Nov | ✅ 21 Nov | — | `testing/subsystem3/logs/2026-11-19_target_test.txt`; robot: `testing/integration/logs/2026-11-21_s1u15_drive_robot.txt` | Done (#38): MQTT map applied in integrated; S1-U15 PASS (A mapped to RIGHT, turned right at the next junction) |

## Subsystem 4: IMU, humps, tilt, lift-stop (Buddy 4)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| LSM303DLHC driver on I2C0 (GP4/GP5) | ✅ | ✅ S4-U01–U02, U04 (Pico) | ⬜ | ⬜ | — | `testing/subsystem3/logs/2026-11-19_target_test.txt` | Claim GP4/GP5 in `TEAM.md` |
| Baseline calibration at start-up | ✅ | ✅ S4-U03 (Pico) | ⬜ | ⬜ | — | `testing/subsystem3/logs/2026-11-19_target_test.txt` | Bench test: keep the robot still for 1 s and log the baseline values |
| Highest hump measurement | 🟡 | ✅ `s4/hump_height` (host, 20 Nov; #5 decided, 255 mm cap now after calibration) | ⬜ | ⬜ | 🟡 20 ms, `all_test` only: 5 mm ramp detected 4/5, 0 false on 2 flat runs (same as the 10 ms build); heights 2–7 mm, mean 4.3 (10 ms build: 4–11, mean 7.8) | `testing/subsystem4/logs/2026-11-20_ramp_rerun_20ms_S4.txt`, `…_rerun_10ms_S4.txt`, `app_program/subsystem4/README.md` (thresholds are placeholders) | Buddy 4: 10 + 10 passes (S4-U09), refit `S4_HEIGHT_CAL_NUM/DEN` at 20 ms if the low bias holds (S4-U10), then repeat in `integrated` (#27). Barcode reading after the GP28 move: A forward 3/3 on 20 Nov (`testing/subsystem3/logs/2026-11-20_barcode_hw.txt`) |
| Tilt flag (used by S5 to ignore readings on ramps); report-only, never stops the robot (#67, 20 Nov) | 🟡 | ✅ INT-03 (host): tilt leaves the robot following | ⬜ | ⬜ | ⬜ | `testing/integration/logs/2026-11-20_host_tests_decisions.txt` | Check it latches **before** the ultrasonic sees the ramp (open risk); INT-11 over real humps (#36) |
| Lift → motors stop (replaces collision detection, removed 13 Nov) | ⬜ | ⬜ | ⬜ | ⬜ | — | — | #53: remove collision, add the lift check, then S4-U12 |
| Motion classes (stationary, accelerating, turning, climbing, descending) | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | #42 |
| Turn rate (magnetometer) | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | #43 |
| Sensing cycle within the sample period (NFR4-1, NFR4-3) | ✅ 20 ms again (20 Nov) | ✅ S4-U05 at 20 ms (Pico, 20 Nov): 0 overruns, max 19 ms (failed at 10 ms on 19 and 20 Nov) | ⬜ | ⬜ | — | `testing/subsystem4/logs/2026-11-20_target_test_20ms.txt` | Re-check humps at 20 ms (S4-U09/U10, #27): the hump filter and confirm counts went from 5 × 10 ms to 3 × 20 ms. Only 1 ms headroom: watch the overrun count |
| Minimum-bit (CR1) version | ⬜ | ⬜ | — | — | — | README says it's the PRE version | Do the CR1 change and log before/after sizes |

## Subsystem 5: Ultrasonic scanning, profiling, avoidance (Buddy 5)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| 1 MHz timer + hardware-timed echo | ✅ | — | ✅ S5-U01 | ⬜ | — | S5 TEST_LOG 2026-10-27 (console not saved) | — |
| Distance accuracy ±2 cm over 5–100 cm (NFR5-6) | ✅ | — | ❌ S5-U02 (before the fix) | ⬜ | — | S5 TEST_LOG 2026-10-27 | **Re-run S5-U02** with the hardware-timed echo; save the log |
| Servo calibration + smooth glide | ✅ | ✅ S5-U21–U22 (Pico) | ✅ S5-U04 | ⬜ | — | S5 TEST_LOG 2026-10-28, `testing/subsystem3/logs/2026-11-19_target_test.txt` | Recheck with the sensor mounted |
| S5 → VC packed request word (IF-10, CR1) | ✅ | ✅ S5-U09 (Pico) | ⬜ | ⬜ | — | `testing/subsystem3/logs/2026-11-19_target_test.txt` | INT-13 host test of the hand-over |
| Coarse scan ≤ 600 ms (NFR5-2) | ✅ | — | ✅ S5-U05 (570 ms, no sensor load) | ⬜ | — | S5 TEST_LOG 2026-10-28 | Retest with the sensor on the horn |
| Detection → action ≤ 2 s (NFR5-3) | ✅ | ⬜ | ❌ S5-U06 2299 ms (before the fix) | ⬜ | — | S5 TEST_LOG 2026-11-01 | **Re-run S5-U06** after the `FINE_BUDGET_POINTS` fix |
| Profiling + planning (S5-U28, U31–U38) | ✅ | ✅ S5-U28, U31–U38 (Pico) | 🟡 | ⬜ | ⬜ | `testing/subsystem3/logs/2026-11-19_target_test.txt`; fixture scans in `testing/subsystem5/logs/2026-11-06_135413_S5_avoidance.txt` | Bench check with the sensor on the horn (S5-U17, U25) |
| Avoid obstacle + reacquire line (FR5-4/5, NFR5-4) | ✅ | ⬜ | 🟡 in all_test only | ⬜ | ⬜ | S5 TEST_LOG INT-S5-01 rows 5–11 Nov (all_test, console only: 29 of 81 obstacles rejoined the line), `testing/subsystem5/logs/*_S5_avoidance.txt` | Ported to integrated 21 Nov with all_test's drive numbers (#8; host-tested only). Run it in `integrated`; record a video; measure the reacquire distance |

---

## Integration (`testing/integration/TEST_LOG.md`)

| Test | Status | Evidence | Next action |
|---|---|---|---|
| INT-01 S2 → S3 samples, INT-02 S3 → S4 summaries | Retired (#74) | — (template example tests; the example tasks are removed) | Write INT tests for the real interfaces in `TEAM.md` §5 |
| INT-03 robot adapter / VC (host, simulated) | ✅ | `logs/2026-11-13_host_tests.txt` (`af2da1c`) | Add a host test of the S5 avoidance hand-over (currently stubbed as idle) |
| INT-14 event-bit table (`TEAM.md` §5, #50) matches the code (host, static) | ✅ | `logs/2026-11-21_event_bits_host.txt` (`882f522` + this change; 8 seeded violations each caught) | Table agreed by the team (21 Nov). S5 bit timing still needs INT-13 (host) and INT-10 (robot); drop COLLISION from the table with #53 |
| INT-04/06 all firmware profiles build | ✅ (28 Oct) | `logs/2026-10-28_robot_builds.txt` | Rerun on the current commit: the `integrated` UF2 is from 8 Nov, older than the latest fixes |
| **Integrated firmware drives the full course on the robot** | ⬜ | — | Main goal: see *Top priorities* below |

## System (`testing/system/TEST_LOG.md`)

| Test | Status | Next action |
|---|---|---|
| SYS tests (one per project requirement) | ⬜ none defined | Write one SYS test per requirement in the brief (line, barcode, hump, obstacle, telemetry) before demo week |

---

## Top priorities (update weekly)

1. **Make `integrated` the real robot.** The all_test tuning (turning,
   driving straight, avoidance handover) is ported into `robot_control.c` /
   `motion_control.c` / `task_motion.c` and host-tested (#8, 21 Nov), with
   the numbers in `subsystem2.h` for both builds. Next: flash `integrated`
   and log a track run.
2. **MQTT**: write the S1 tests and get a bench run with a real broker.
3. **Barcode**: test all four commands at driving speed.
4. **S4 thresholds** from real hump logs.
5. Re-run the failed S5 tests (S5-U02, S5-U06).

## Housekeeping

- [ ] Fill in `TEAM.md` (owners, GitHub usernames, pins, interfaces). It still has the template's example rows.
- [ ] One shared build script with relative paths and `PICO_SDK_PATH` from the environment (#11). `sh build.sh <mode>` replaces the old per-machine `build_*_current.sh` scripts; tick this once every buddy has built `integrated` with it.
- [x] Delete the duplicate copies with " 2" in the name (done 2026-11-13: 8 files removed, all identical to or older than the originals).
- [x] Stop committing build output: test `.exe` files and build `.log` files are untracked and gitignored (2026-11-13).
- [x] One host-test runner: `sh testing/run_host_tests.sh` (2026-11-13).
- [x] Host tests: 13 of 13 pass (21 Nov; `int/event_bits` added for #50). The 6 `int/all_buddies*` tests were retired (#9) now that `integrated` is final. The rest need their owners:
  - ~~**zhi xian:** push the missing local code~~. Resolved 20 Nov (#3 closed): the code was lost. `s1/barcode_queue` passes against a reimplementation on branch `s1/barcode-queue`; `int/steering_ramp` no longer checks terrain boost or barcode slow-down (not requirements) and passes.
- [x] S1 barcode-over-MQTT code: reimplemented 20 Nov on branch `s1/barcode-queue`, with a binary payload (#3).
- [ ] Choose one integration branch (`alicia` or `double11`), merge into it, and delete the dead branches (`zhixin`, `integration`, `master`, …).
- [x] Commit the untracked S5 console logs and link them from the test-log result rows (2026-11-20: trimmed copies `testing/subsystem5/logs/*_S5_avoidance.txt`; the raw `*_console.txt` stay gitignored; #28).

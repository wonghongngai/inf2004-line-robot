# Project status

**This is the one page that shows what is done and what is left.**
What to test next, and how, is in [testing/TEST_PLAN.md](testing/TEST_PLAN.md).
Last updated: 2026-11-13 · Branch: `double11` @ `1d78677`

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
| 80-bit telemetry packing (CR1) | ✅ | ⬜ | ⬜ | ⬜ | — | — | Replace the template S1 test cases (LED tests) with packing/range tests; run `make TEST=1` and log them |
| MQTT connect + reconnect after AP or broker loss | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | — | Define S1-U tests (connect, publish rate, reconnect time); run on the bench with a broker and save logs |
| Telemetry publish (state, speed, hump, obstacle…) | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | — | Check every field decodes correctly on the subscriber side; record the publish rate |
| Barcode result events (`<id>/barcode`, QoS 1) | 🟡 | 🟡 host test exists | ⬜ | ⬜ | ⬜ | `testing/subsystem1/BARCODE_MQTT.md` (no result logged) | Run `barcode_queue_test.c` and log the result; then do a bench test with a real broker |
| Letter-to-movement map received over MQTT and **applied** | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | Both builds hard-code the map today; all_test ignores MQTT maps | #38: apply the MQTT map (go straight until one arrives), then S1-U15 and S3-U27 |

## Subsystem 2: Motors, encoders, PID, Vehicle Controller (Buddy 2)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| Motor PWM driver | ✅ | ✅ INT-05 | 🟡 | ⬜ | — | `testing/integration/logs/2026-10-28_pwm_fix.txt` | Flash `motor_test` with the wheels raised, follow `HARDWARE_TEST.md`, log the result |
| Encoders + odometry | ✅ | ⬜ | 🟡 | ⬜ | ⬜ | `CALIBRATION.md`: old calibration not validated for IRQ counts | Recalibrate counts per cm and per degree; log them |
| Speed PID | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | `CALIBRATION.md`: gains still 0 | Tune the gains on the robot; record step response / logs |
| Straight driving + turning | 🟡 | ⬜ | 🟡 in all_test only | ⬜ | ⬜ | Commits `ab62be5`, `1d78677` (no test log) | **Port the all_test tuning into `robot_control.c` / `task_motion.c`**, then retest in integrated |
| Compact motion protocol v1 (CR1) | ✅ | 🟡 host test exists | ⬜ | ⬜ | — | `TRIBUNAL_STATUS.md` ("physical tests remain pending") | Run `motion_protocol_host_test.c` and log it; move the other subsystems onto `subsystem2_send_command` |
| Vehicle Controller mode arbitration (safety > avoidance > line) | 🟡 | ✅ INT-03 (simulated) | ⬜ | ⬜ | ⬜ | `testing/integration/logs/2026-10-28_robot_host.txt` | Bench test with real S3/S4/S5 inputs |

## Subsystem 3: IR line following + barcode (Buddy 3)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| Line position from 3 IR sensors, noise rejection | ✅ | ✅ S3-U02–U03 | ⬜ | ⬜ | ⬜ | `testing/subsystem3/logs/2026-10-27_cr1_host.txt` | Run S3-U14–U17 on target; calibrate black/white on the real track |
| Line loss → search → reacquire / fault | ✅ | ✅ S3-U04–U06 | ⬜ | ⬜ | ⬜ | same log | Test on the track: lift the robot off the line, time how long reacquiring takes |
| Code 39 decode A–Z, forward + backward | ✅ | ✅ S3-U07–U13 | 🟡 only A and Z tested | ⬜ | ⬜ | Commit `208188b` (no test log); all A–Z both ways required (prof, 13 Nov) | Test all four command barcodes at real driving speed; log the decode rate per letter |
| Junction detection + barcode → command at next junction | ✅ | ✅ S3-U25–U26 | ⬜ | ⬜ | ⬜ | `testing/subsystem3/logs/2026-10-26_junction_host.txt` | Track test at each junction type |
| Run-time letter mapping (S3-U27) | ✅ | ⬜ PENDING | ⬜ | ⬜ | — | — | Run on target together with S1 |

## Subsystem 4: IMU, humps, tilt, lift-stop (Buddy 4)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| LSM303DLHC driver on I2C0 (GP4/GP5) | ✅ | ⬜ | ⬜ | ⬜ | — | — | Claim GP4/GP5 in `TEAM.md`; replace the template S4 test cases with IMU ones |
| Baseline calibration at start-up | ✅ | ⬜ | ⬜ | ⬜ | — | — | Bench test: keep the robot still for 1 s and log the baseline values |
| Highest hump measurement | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | `README_BUDDY4.txt`: thresholds are placeholders | Collect IMU logs over real humps; set the thresholds from that data |
| Tilt flag (used by S5 to ignore readings on ramps) | 🟡 | ⬜ | ⬜ | ⬜ | ⬜ | — | Check it latches **before** the ultrasonic sees the ramp (open risk) |
| Lift → motors stop (replaces collision detection, removed 13 Nov) | ⬜ | ⬜ | ⬜ | ⬜ | — | — | #53: remove collision, add the lift check, then S4-U12 |
| Motion classes (stationary, accelerating, turning, climbing, descending) | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | #42 |
| Turn rate (magnetometer) | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | #43 |
| Minimum-bit (CR1) version | ⬜ | ⬜ | — | — | — | README says it's the PRE version | Do the CR1 change and log before/after sizes |

## Subsystem 5: Ultrasonic scanning, profiling, avoidance (Buddy 5)

| Feature | Code | Host/Unit | Bench HW | Integrated | Track tuned | Evidence | Next action |
|---|---|---|---|---|---|---|---|
| 1 MHz timer + hardware-timed echo | ✅ | — | ✅ S5-U01 | ⬜ | — | S5 TEST_LOG 2026-10-27 (console not saved) | — |
| Distance accuracy ±2 cm over 5–100 cm (NFR5-6) | ✅ | — | ❌ S5-U02 (before the fix) | ⬜ | — | S5 TEST_LOG 2026-10-27 | **Re-run S5-U02** with the hardware-timed echo; save the log |
| Servo calibration + smooth glide | ✅ | 🟡 S5-U21–U22 defined | ✅ S5-U04 | ⬜ | — | S5 TEST_LOG 2026-10-28 | Run the S5-U21/U22 auto tests and log them; recheck with the sensor mounted |
| Coarse scan ≤ 600 ms (NFR5-2) | ✅ | — | ✅ S5-U05 (570 ms, no sensor load) | ⬜ | — | S5 TEST_LOG 2026-10-28 | Retest with the sensor on the horn |
| Detection → action ≤ 2 s (NFR5-3) | ✅ | ⬜ | ❌ S5-U06 2299 ms (before the fix) | ⬜ | — | S5 TEST_LOG 2026-11-01 | **Re-run S5-U06** after the `FINE_BUDGET_POINTS` fix |
| Profiling + planning (S5-U31–U38) | ✅ | ⬜ cases defined, no results | 🟡 | ⬜ | ⬜ | Console logs 2026-11-05…11-11 in `testing/subsystem5/logs/` (not yet linked to results) | Run `make TEST=1`; add result rows; commit the untracked logs |
| Avoid obstacle + reacquire line (FR5-4/5, NFR5-4) | ✅ | ⬜ | 🟡 in all_test only | ⬜ | ⬜ | Commits `a942e14`, `ab62be5` (no test log) | Port to the integrated path; record a video; measure the reacquire distance |

---

## Integration (`testing/integration/TEST_LOG.md`)

| Test | Status | Evidence | Next action |
|---|---|---|---|
| INT-01 S2 → S3 samples, INT-02 S3 → S4 summaries | ⬜ | — (these are still the template's example tests) | Replace them with the real interfaces from `TEAM.md` §5 |
| INT-03 robot adapter / VC (host, simulated) | ✅ | `logs/2026-11-13_host_tests.txt` (`af2da1c`) | Add a host test of the S5 avoidance hand-over (currently stubbed as idle) |
| INT-04/06 all firmware profiles build | ✅ (28 Oct) | `logs/2026-10-28_robot_builds.txt` | Rerun on the current commit: the `integrated` UF2 is from 8 Nov, older than the latest fixes |
| **Integrated firmware drives the full course on the robot** | ⬜ | — | Main goal: see *Top priorities* below |

## System (`testing/system/TEST_LOG.md`)

| Test | Status | Next action |
|---|---|---|
| SYS tests (one per project requirement) | ⬜ none defined | Write one SYS test per requirement in the brief (line, barcode, hump, obstacle, telemetry) before demo week |

---

## Top priorities (update weekly)

1. **Make `integrated` the real robot.** Move the all_test tuning (turning,
   driving straight, avoidance handover) out of `all_buddies_test.c` into
   `robot_control.c` / `task_motion.c`. Rebuild, flash, and log the result
   (#8). Keep the tuning numbers in one place both builds use, so they can't
   drift apart again.
2. **MQTT**: write the S1 tests and get a bench run with a real broker.
3. **Barcode**: test all four commands at driving speed.
4. **S4 thresholds** from real hump logs.
5. Re-run the failed S5 tests (S5-U02, S5-U06).

## Housekeeping

- [ ] Fill in `TEAM.md` (owners, GitHub usernames, pins, interfaces). It still has the template's example rows.
- [ ] One shared build script with relative paths and `PICO_SDK_PATH` from the environment. The current `build_make/build_*.sh` scripts hardcode one teammate's machine.
- [x] Delete the duplicate copies with " 2" in the name (done 2026-11-13: 8 files removed, all identical to or older than the originals).
- [x] Stop committing build output: test `.exe` files and build `.log` files are untracked and gitignored (2026-11-13).
- [x] One host-test runner: `sh testing/run_host_tests.sh` (2026-11-13).
- [ ] Host tests: 9 of 18 pass (`af2da1c`). The rest need their owners:
  - **zhi xian:** push the missing local code. `s1/barcode_queue` (barcode-over-MQTT queue) and `int/steering_ramp` (`update_terrain_boost`, `barcode_slow`) test code that isn't on any branch. Both tests came in with `673736f`.
  - **Buddy 2 (all_test tuning):** `int/all_buddies` + 4 variants now compile and run, but expect the old PWM tuning (right 14.2 %, centre = straight); the code now gives 15 % and centre → 6.25/15. Update the expected values or the code. `int/all_buddies_bal` uses `straight_error_cm` / `ALL_TEST_STRAIGHT_MAX_TRIM`, which the straight-drive rewrite (`ab62be5`) removed.
  - **Buddy 4:** `s4/hump_height` expects the height to fall back to 0 after the descent. Since `7f823b8` the estimator deliberately keeps the climb height. Decide which is right and update the test or the code.
- [ ] S1 barcode-over-MQTT code (`S1_BARCODE_RESULT`, barcode queue) isn't on any branch. Only its test and doc were committed (`673736f`). Ask zhi xian to push it.
- [ ] Choose one integration branch (`alicia` or `double11`), merge into it, and delete the dead branches (`zhixin`, `integration`, `master`, …).
- [ ] Commit the untracked S5 console logs and link them from the test-log result rows.

# Subsystem 2 — test log

**Subsystem:** Motors, encoders, PID, Vehicle Controller  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

**Rewritten 20 Nov (#17).** Until then this table still described the course
template's sample/pool tests under S2-U01 and U02. Those were never run, so no
results are lost; the IDs now describe what `subsystem2_test.c` checks. The
on-target checks never move the motors: they drive `motion_control.c` with
simulated encoder counts and a test-only configuration (10 ticks/cm, 100 ms
stall timeout), not the robot's calibration.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | The motion command message buffer is sized for one `MOTION_COMMAND` | IF-09 | auto | `make TEST=1`: `tk_ref_mbf(mbf_motion_cmd)`, compare `maxmsz` with `sizeof(MOTION_COMMAND)` | `[TEST] S2-U01 PASS` |
| S2-U02 | A FORWARD 10 cm command completes when the simulated encoders reach the target, and both motor outputs go to zero | FR2-2, FR2-4 | auto | `make TEST=1`: FORWARD_CM 10, then steps at 50 and 100 ticks per wheel | state IDLE, `motion_complete`, PWM 0 / 0 |
| S2-U03 | STOP during a move zeros both motor commands at once | FR2-4 | auto | `make TEST=1`: FORWARD_CM 50, one step at 10 ticks, then STOP | state STOPPING, PWM 0 / 0 |
| S2-U04 | Encoders that do not change while a move is commanded put the controller in FAULT and stop both motors | FR2-1 | auto | `make TEST=1`: FORWARD_CM 20, then one step at 101 ms with both encoders still 0 (stall timeout 100 ms) | state FAULT, fault set, PWM 0 / 0 |
| S2-U05 | Motor direction and sides (#18) | FR2-1 | manual | `sh build.sh motor_test`, wheels raised, steps in `HARDWARE_TEST.md`; each stage runs one or both wheels at 20 % for 750 ms | M1 = left, M2 = right, both turn forward; the idle wheel's ticks stay near 0 |
| S2-U06 | Encoder ticks per cm, each wheel (#18) | FR2-2 | manual | Push or drive 100 cm three times on the floor; read left and right ticks per run | spread between runs ≤ 2 % per wheel; means written to `CALIBRATION.md` and `subsystem2.h` |
| S2-U07 | Ticks per degree, pivot (#18) | FR2-3 | manual | 4 × 90° and 1 × 360° pivots, compare with floor marks | within ±10° per 360° |
| S2-U10 | Straight-line PID response (#18; retargeted from "speed PID" because the controller has no speed loop) | FR2-1 | manual | `make ROBOT_MODE=motion MOTION_DEMO=1 PID_BENCH=1`, robot on the floor with 1.2 m clear; run 1 drives 100 cm and logs left-minus-right travel (`err_mm`) and PWM every 20 ms. Run once with gains 0 (baseline), then with tuned `S2_PID_*` | tuned: `\|err_mm\|` ≤ 10 from 300 ms to the end, correction not pinned at the limit for > 300 ms; baseline log kept for comparison |
| S2-U11 | STOP latency (#18) | FR2-4 | manual | STOP command to wheels stopped. Same build as S2-U10; run 2 sends STOP 1 s into a 100 cm run through `subsystem2_send_command`. The firmware logs `motors off at +N ms` and `wheels still at +N ms` (1 ms loop, ±2 ms). Optional cross-check: slow-motion video | `wheels still` ≤ 100 ms, in 3 repeats |

Host (PC) tests, run with `sh testing/run_host_tests.sh s2/` and
`sh testing/run_host_tests.sh int/robot_control`:

| Name | What it checks |
|---|---|
| `s2/motion_protocol` | compact command/status bytes: layout, reserved values, numeric boundaries and precision |
| `s2/motor_pwm` | PWM driver on modelled registers: reset, counters, outputs, faults (INT-05) |
| `int/robot_control` | Vehicle Controller line/barcode/IMU logic and line-stop re-arm, 209 checks (INT-03; its results are kept in `integration/TEST_LOG.md`) |

S2-U05–U07 and U11 (#18) are defined above. S2-U10 is retargeted to the
straight-line PID because the controller has no speed PID; the track tests (S2-U08, U09,
U12, U13) are in `TEST_PLAN.md`, Buddy 2, and get added here before running.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S2-U01–U04 **on target** | `d9c4830` | robot operator; log analysis Claude | PASS (4 checks, 0 failed) | [`../subsystem3/logs/2026-11-19_target_test.txt`](../subsystem3/logs/2026-11-19_target_test.txt) | `robot-integrated-test.uf2` (`sh build.sh integrated TEST=1`), boot 22:29; the full test run for all subsystems. First logged run of the S2 checks. `subsystem2 SUMMARY 4 passed, 0 failed` |
| 2026-11-20 | `s2/motion_protocol`, `s2/motor_pwm` | `65ba452` | Claude, host | PASS | [`logs/2026-11-20_host.txt`](logs/2026-11-20_host.txt) | `sh testing/run_host_tests.sh -v s2/` on a clean tree. Host only; no sanitizers |
| 2026-11-20 | `int/robot_control` | `65ba452` | Claude, host | PASS (209 checks) | [`logs/2026-11-20_host.txt`](logs/2026-11-20_host.txt) | `sh testing/run_host_tests.sh -v int/robot_control`. Recorded here for #17; it is INT-03 |
| 2026-11-20 | S2-U01–U04 **on target**, Wi-Fi on and off | `65ba452` + uncommitted `chore/m0-housekeeping` changes | Claude (flash + capture); robot on a stand, set up by the user | PASS (4 checks each run) | [`../subsystem4/logs/2026-11-20_target_test.txt`](../subsystem4/logs/2026-11-20_target_test.txt), [`../subsystem4/logs/2026-11-20_target_test_offline.txt`](../subsystem4/logs/2026-11-20_target_test_offline.txt) | Same runs as the S1-U01 re-test (#14) |
| 2026-11-20 | S2-U01–U04 **on target**, 20 ms IMU build | `65ba452` + uncommitted `chore/m0-housekeeping` changes (20 ms period) | Claude (flash + capture); robot on a stand, set up by the user | PASS (4 checks) | [`../subsystem4/logs/2026-11-20_target_test_20ms.txt`](../subsystem4/logs/2026-11-20_target_test_20ms.txt) | Same run as the S4 20 ms re-test |
| 2026-11-20 | S2-U01–U04 **on target** | `c27ab81` + uncommitted #74 changes (`s2/remove-template-tasks`) | Claude (flash + capture); robot on a stand, set up by the user | PASS (4 checks, 0 failed) | [`../integration/logs/2026-11-20_target_test_74.txt`](../integration/logs/2026-11-20_target_test_74.txt) | `sh build.sh integrated TEST=1`, flashed 19:28. First run without the template `producer_task` (#74): the firmware boots and starts motion with no example task |

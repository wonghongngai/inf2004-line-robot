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

Host (PC) tests, run with `sh testing/run_host_tests.sh s2/` and
`sh testing/run_host_tests.sh int/robot_control`:

| Name | What it checks |
|---|---|
| `s2/motion_protocol` | compact command/status bytes: layout, reserved values, numeric boundaries and precision |
| `s2/motor_pwm` | PWM driver on modelled registers: reset, counters, outputs, faults (INT-05) |
| `int/robot_control` | Vehicle Controller line/barcode/IMU logic and line-stop re-arm, 209 checks (INT-03; its results are kept in `integration/TEST_LOG.md`) |

The bench and track tests (S2-U05 onwards) are defined in `TEST_PLAN.md`,
Buddy 2; add them here before running them.

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

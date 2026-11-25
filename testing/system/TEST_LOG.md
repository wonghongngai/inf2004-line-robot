# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

Run these cases on the final hardware using the `integrated` firmware. The
[system test plan](../TEST_PLAN.md#system-testingsystem-the-whole-robot-against-the-brief)
defines the acceptance criteria.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Full-course navigation | System mission, FR3-1 | Manual track test | Place the robot at the course start, load the MQTT letter map, and start the run. Repeat to obtain three consecutive runs. | The robot completes all three runs from start to finish without being touched. |
| SYS-02 | Barcode-directed navigation | FR3-3, IF-07, IF-12 | Track observation and logs | In the SYS-01 runs, record each decoded letter, its configured action, and the action taken at the next cross-junction. | Every barcode on the course produces the correct action at the next cross-junction. |
| SYS-03 | Highest hump height over MQTT | FR4-2 | Dropped | The final track has no hump. See [S4-U09 and S4-U10](../subsystem4/TEST_LOG.md) for the separate ramp tests. | Dropped on 23 November 2026. |
| SYS-04 | Obstacle bypass and line recovery | FR5-4, FR5-5 | Track observation and logs | In the SYS-01 runs, observe the robot as it approaches the obstacle, bypasses it, and rejoins the guide line. | The robot passes the obstacle without contact, reacquires the guide line, and resumes following it. |
| SYS-05 | Telemetry throughout a full run | FR1-2, IF-05 | Timestamped MQTT capture | Capture telemetry from the start to the finish of a SYS-01 run. Record the longest interval between packets and the final reported state. | No telemetry gap exceeds 3 s. The final reported state is `FINISHED`. |
| SYS-06 | Full-course demonstration video | Submission evidence | Manual video review | Record one uncut SYS-01 run with the console or MQTT dashboard visible. Add the unlisted YouTube link to the [video index](../../videos/README.md). | The linked video shows the complete run and the console or MQTT view. |

The `FINISHED` acceptance criterion for SYS-05 comes from the test plan. The
current `integrated` firmware does not automatically report `FINISHED` at the
end of the course. Record the actual final state in the result.

## Results

The team tracks full-course verification in [issue #37](https://github.com/wonghongngai/inf2004-line-robot/issues/37).

### Course recording, 25 November 2026

Evidence for these observations is the 2 min 45 s local recording `IMG_3433.MOV`.
Times are offsets into the video. The flashed firmware revision is unknown.

| Test ID | Recorded result | Video evidence |
|---|---|---|
| `SYS-01` | **One run observed.** The robot reversed and re-approached the first barcode twice, then continued along the course. The clip ends with the robot on the guide line by another set of barcodes. The designated finish is unclear. The test requires three consecutive completions. | 00:19–02:45. First barcode re-approaches at about 00:28–00:46. |
| `SYS-02` | **Two junction actions matched the displayed map.** The robot crossed straight after the printed `Z` and turned left after the printed `A`. Those are the two actions matched to labels in the footage. The robot’s decoded-letter output was off-screen during the run, so the straight crossing alone cannot confirm a successful decode. | Dashboard map at 00:13–00:15. `Z` approach and straight crossing at 00:48–00:56. `A` approach and left turn at 01:24–01:34. |
| `SYS-04` | **One bypass and line recovery observed.** The robot stopped short of the orange obstacle, went around it with no visible contact, rejoined the guide line, and followed the next bends. | Stopped at about 01:50. Bypass at about 01:56–02:20. Back on the guide line by 02:22. |
| `SYS-05` | **Startup telemetry visible.** The dashboard showed `Idle`, no reported faults, and a telemetry receive time. The longest gap and final telemetry state could not be measured because the dashboard went off-screen before the track run. | Dashboard at about 00:15. |
| `SYS-06` | **Local recording reviewed.** One continuous clip shows the dashboard before the camera follows the robot on the course. The designated finish is unclear. The video index still needs the unlisted link. | 00:00–02:45. |

### Supporting hardware results

These results retain their original test IDs and scopes.

| Date | Test ID | Firmware revision | Recorded result | Evidence |
|---|---|---|---|---|
| 2026-11-22 | `INT-08` | `49952fc` with local tuning changes | **PASS.** The tester counted 21 of 21 correct mapped manoeuvres across separate junction trials. Two left-turn trials stopped for 3–9 s after turning. | [Integration test log](../integration/TEST_LOG.md#results) |
| 2026-11-23 | `S3-U29` | `f399b56` with local line-following changes | **PASS.** In five straight-track runs of at least 3 m, the robot stayed on the guide line. | [Subsystem 3 test log](../subsystem3/TEST_LOG.md#results) |
| 2026-11-23 | `INT-10`, `S5-U41`, `S5-U42` | `5fd4da5` and `ea03e1a` | **PASS.** The robot passed the box without contact and resumed line following in all nine scored runs. It reacquired the line after about 30 cm on the return leg. | [Subsystem 5 test log](../subsystem5/TEST_LOG.md#results) |
| 2026-11-23 | `S1-U12` | `fb8d46c` with local telemetry changes | **PASS.** The stationary robot published 300 valid packets in five minutes. The longest gap was 2.20 s. | [MQTT test capture](../integration/logs/2026-11-23_S1-U12_19byte_robot_mqtt_test.txt) |

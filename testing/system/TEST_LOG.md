# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

Run these cases on the final hardware using the `integrated` firmware. The
[system test plan](../TEST_PLAN.md#system-testingsystem-the-whole-robot-against-the-brief)
defines the acceptance criteria.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Full-course navigation | System mission, FR3-1 | Manual track test | Place the robot at the course start, load the MQTT letter map, and start the run. Repeat to obtain three consecutive runs. | The robot completes all three navigation runs without intervention; operator PAUSE or lift-stop terminates each completed run. |
| SYS-02 | Barcode-directed navigation | FR3-3, IF-07, IF-12 | Track observation and logs | In the SYS-01 runs, record each decoded letter, its configured action, and the action taken at the next cross-junction. | Every barcode on the course produces the correct action at the next cross-junction. |
| SYS-03 | Highest hump height over MQTT | FR4-2 | Track measurement and MQTT capture | Measure the pink hump identified by the operator and compare its height with the highest hump reported over MQTT. See S4-U10 for the tolerance. | Reopened 25 November 2026; Manual observation: physical height 10 mm. The full-run highest-height telemetry remains unverified. |
| SYS-04 | Obstacle bypass and line recovery | FR5-4, FR5-5 | Track observation and logs | In the SYS-01 runs, observe the robot as it approaches the obstacle, bypasses it, and rejoins the guide line. | The robot passes the obstacle without contact, reacquires the guide line, and resumes following it. |
| SYS-05 | Telemetry throughout a full run | FR1-2, IF-05 | Timestamped MQTT capture | Capture telemetry from the start to the finish of a SYS-01 run. Record the longest interval between packets and the final reported state. | No telemetry gap exceeds 3 s through operator PAUSE or lift-stop at completion. Record the actual final telemetry state and termination method; automatic `FINISHED` is not required. |
| SYS-06 | Full-course demonstration video | Submission evidence | Manual video review | Record one uncut SYS-01 run with the console or MQTT dashboard visible. Add the unlisted YouTube link to the [video index](../../videos/README.md). | The linked video shows the complete run and the console or MQTT view. |

Completion is declared by the operator using PAUSE or by lifting the robot
(lift-stop), as confirmed on 25 November 2026. These terminal actions do not
count as navigation intervention. Record the actual telemetry state;
`integrated` does not automatically report `FINISHED`. The pink strip is a
hump, not a finish marker.

## Results

The team tracks full-course verification in [issue #37](https://github.com/wonghongngai/inf2004-line-robot/issues/37).

### Course recording, 25 November 2026

Evidence for these observations is the 2 min 45 s local recording `IMG_3433.MOV`.
Times are offsets into that original recording. The flashed firmware revision
is unknown.

| Test ID | Recorded result | Video evidence |
|---|---|---|
| `SYS-01` | **One run observed.** The robot reversed and re-approached the first barcode twice, then continued along the course. The clip ends with the robot on the guide line by another set of barcodes. The designated finish is unclear. The test requires three consecutive completions. | 00:19–02:45. First barcode re-approaches at about 00:28–00:46. |
| `SYS-02` | **Two junction actions matched the displayed map.** The robot crossed straight after the printed `Z` and turned left after the printed `A`. Those are the two actions matched to labels in the footage. The robot’s decoded-letter output was off-screen during the run, so the straight crossing alone cannot confirm a successful decode. | Dashboard map at 00:13–00:15. `Z` approach and straight crossing at 00:48–00:56. `A` approach and left turn at 01:24–01:34. |
| `SYS-04` | **One bypass and line recovery observed.** The robot stopped short of the orange obstacle, went around it with no visible contact, rejoined the guide line, and followed the next bends. | Stopped at about 01:50. Bypass at about 01:56–02:20. Back on the guide line by 02:22. |
| `SYS-05` | **Startup telemetry visible.** The dashboard showed `Idle`, no reported faults, and a telemetry receive time. The longest gap and final telemetry state could not be measured because the dashboard went off-screen before the track run. | Dashboard at about 00:15. |
| `SYS-06` | **Full-track video linked.** The original clip shows the dashboard before the camera follows the robot on the course. The designated finish is unclear. | [Full-track upload](https://youtube.com/shorts/0AZ4yvPSE0g?feature=share). Original recording at 00:00–02:45. |

### Supporting hardware results

These results retain their original test IDs and scopes.

| Date | Test ID | Firmware revision | Recorded result | Evidence |
|---|---|---|---|---|
| 2026-11-22 | `INT-08` | `49952fc` with local tuning changes | **PASS.** The tester counted 21 of 21 correct mapped manoeuvres across separate junction trials. Two left-turn trials stopped for 3–9 s after turning. | [Integration test log](../integration/TEST_LOG.md#results) |
| 2026-11-23 | `S3-U29` | `f399b56` with local line-following changes | **PASS.** In five straight-track runs of at least 3 m, the robot stayed on the guide line. | [Subsystem 3 test log](../subsystem3/TEST_LOG.md#results) |
| 2026-11-23 | `INT-10`, `S5-U41`, `S5-U42` | `5fd4da5` and `ea03e1a` | **PASS.** The robot passed the box without contact and resumed line following in all nine scored runs. It reacquired the line after about 30 cm on the return leg. | [Subsystem 5 test log](../subsystem5/TEST_LOG.md#results) |
| 2026-11-23 | `S1-U12` | `fb8d46c` with local telemetry changes | **PASS.** The stationary robot published 300 valid packets in five minutes. The longest gap was 2.20 s. | [MQTT test capture](../integration/logs/2026-11-23_S1-U12_19byte_robot_mqtt_test.txt) |


### Reviewing full track demo video, reviewed 25 November 2026

[Public video](https://youtube.com/shorts/0AZ4yvPSE0g?feature=share).

- SYS-01: Manual observation: one whole take with smooth continuous
  movement and no stops. Three consecutive completions are not established;
  the terminal PAUSE/lift action and firmware revision remain unconfirmed.
- SYS-02: decoded-letter output and the full configured map are not
  established for every junction by the sampled frames.
- SYS-04: bypass and line recovery are visible qualitatively; clearance
  and recovery distance are not measured.
- SYS-05: no timestamped MQTT capture throughout the run is supplied.
  Maximum packet gap and final state remain unverified. Completion is
  operator PAUSE or lift-stop; automatic FINISHED is not required.
- SYS-06: Manual observation: this is one whole take. The earlier
  inference of edits from sampled frames was incorrect. Continuous
  console/MQTT visibility is not established by this footage review.

Recording date and flashed firmware mode/revision await confirmation.

### SYS-01 acceptance result

| Date | Test ID | Firmware | Tester / evidence | Result | Notes |
|---|---|---|---|---|---|
| 2026-11-24 (operator-supplied documentation date; exact recording date unknown) | SYS-01 | integrated; flashed commit unknown | Manual observation: at least three complete course laps consecutively without touching, repositioning or resetting the robot during navigation; one lap filmed in full_track.mp4 | **PASS** | Confirmation supplied 25 Nov 2026. Terminal PAUSE or lift-stop after completion is allowed. Only one lap has video evidence; the three-lap count rests on operator testimony. This supersedes the earlier unverified-count status without changing the earlier observations. |

### Final acceptance results, recorded 25 November 2026

Test date: 24 November 2026; firmware: integrated. These results supersede earlier unverified statuses. The tester confirms that all listed criteria were
observed. No full-run MQTT capture was saved; no exact maximum gap or
exact full-track height value was supplied.

| Test ID | Result | Acceptance observations | Evidence / limitation |
|---|---|---|---|
| SYS-03 | **PASS** | The full-track dashboard highest-hump reading was within 7-13 mm against the physical 10 mm hump, meeting +/-3 mm. | Reported test observations; exact full-track reading not recorded. The separate S4 video shows 9 mm. |
| SYS-05 | **PASS** | The full-run maximum MQTT packet gap was checked and met the <=3 s limit; the final state matched the termination method (IDLE after PAUSE or FAULT after lift-stop). | Reported test observations; full-run capture, exact maximum gap and specific termination method were not saved in this record. |

These are manual acceptance results. 
# Team decisions

The decisions that settle a deviation from the Week 6 / PreTri plan, with the
reason for each, so it can be defended at Tribunal 2 (#63). Where the code
differs from the plan is in [DEVIATIONS.md](DEVIATIONS.md); what is done is in
[STATUS.md](STATUS.md). Add a section here for every new decision.

Decided 13 Nov 2026: `integrated` is the final firmware (#4, see `STATUS.md`).

## 22 Nov 2026: humps and the ultrasonic

| Issue | Decision | Follow-up |
| --- | --- | --- |
| #44 | **S5 does not gate on the IMU tilt state** (A5-2 dropped): the hump is too small to matter | S5-U44 dropped in `TEST_PLAN.md` |
| #36 | **INT-11 dropped:** the ultrasonic sits above the hump's height, so it cannot see the hump | — |
| #34 | INT-09 target cut to 5/5 lifts; tilt removed (report-only, #67) | PASS on the robot 21 Nov (S4-U12) |

## 20 Nov 2026: M1 decisions

Decided by the team on 20 Nov 2026, from `main` @ `42be6ea`.

| Issue | Decision | Follow-up |
| --- | --- | --- |
| #5 | S4 keeps the climb height after a hump. The test was changed, not the firmware | — |
| #6 | Every `[set]` threshold in `TEST_PLAN.md` agreed (table below) | — |
| #48 | **Add a START command.** The robot waits in Ready until START arrives | Built and merged 21 Nov (`234f2bb`): `S1_COMMAND_START` (code 5) + VC Ready state; `S2_AUTO_START` fallback. Robot test PASS 21 Nov (integration TEST_LOG) |
| #38 | **No on-robot A–Z print test** (decided 21 Nov, time). A–Z in both directions rests on S3-U07 (all 26 letters, synthetic frames, host + Pico) and S3-U18 (real PDF A/Z); on the robot only A was scanned (large print 3/3 forward, 3/3 reversed; medium print failed reversed) | Use **large prints** on demo day; the medium-print failure is a known risk |
| #72 | **No built-in letter map; a junction without a mapped barcode goes straight** (decided 21 Nov), before and after an MQTT map arrives, as in `all_test` | Built in `robot_control.c`; host-tested in `int/robot_control`. all_test's stop on an incomplete barcode (scanner edges, no letter) is **not** ported to integrated (decided 21 Nov): integrated goes straight there too |
| #54 | Obstacle distance is 9-bit cm on the wire, mm inside S5 | VC converts `mm / 10` when filling IF-05 (#72) |
| #64 | Keep 20 cm detection | Buddy 5 records the reason; S5-U40 (#31) must show the robot rests ≥ 10 cm away |
| #65 | Keep the 50–130° scan. FR5-1 changed to 50, 70, 90, 110, 130° | Re-measure NFR5-2 with the sensor mounted (#29) |
| #66 | Keep the current task priorities | S1-U17 (#15) and S2-U12 (#19) measure loop lateness with Wi-Fi on |
| #67 | Tilt is report-only | INT-11 (#36) dropped 22 Nov: see below |
| #68 | On line loss the robot stops and waits. Statechart 3 changed | S3-U31 (#23) |

### #5 Hump height after the descent

**Decision.** S4 reports the **highest** rise, and the descent does not reduce
it. The host test `s4/hump_height` no longer checks `current_height_mm` after
the descent.

**Why.** The brief asks for the highest hump. `current_height_mm` is used only
inside `task_imu.c` and is never published. What leaves S4 is the highest
hump (`last_ramp_height`). Leaving the estimator alone avoids disturbing
Buddy 4's tuning (S4-U10).

**Found while fixing the test.** The test also had to include the 25/11
height calibration factor added on 19 Nov (`7039135`). The 255 mm cap was
applied before that factor, so a reported height could reach 579 mm, which
overflows telemetry's 8-bit field. The cap now applies after calibration
(`calibrated_height_mm()` in `task_imu.c`).

### #6 Pass thresholds

| Test | Threshold | Why |
| --- | --- | --- |
| S1-U11 Wi-Fi join | ≤ 15 s, 5/5 | Wi-Fi only affects telemetry |
| S1-U12 publish gap | no gap > 3 s | 1 packet/s, so 3 s means two packets lost in a row |
| S1-U13 reconnect | ≤ 10 s after the broker or AP is back | Several reconnect attempts fit in 10 s |
| S1-U14 PAUSE | stops within 500 ms | MQTT round trip, plus one 10 ms VC cycle, plus braking |
| S2-U07 pivot | ±10° per 360° | Barcode turns end in line following, which corrects small errors |
| S2-U08 straight | ≤ 10 cm drift over 2 m | 5 %. Line following corrects it on the track |
| S2-U09 turn angles | ±15°, line reacquired 10/10 | What matters is that the line is reacquired |
| S2-U10 PID step | settles ≤ 300 ms, overshoot ≤ 20 % | Starting values. The gains are 0 today |
| S2-U11 STOP | ≤ 100 ms | 10 ms control loop plus the motor run-down |
| S2-U13 reverse | ±2 cm | Same as the S5 distance accuracy (NFR5-6) |
| S3-U31 reacquire | S3 FOUND ≤ 200 ms after the line is back; VC re-armed ≤ 1.5 s | The VC waits `S2_LINE_REARM_MS` = 1 s with the wheels still before it re-arms |
| S3-U32 letters | ≥ 9/10 per letter | |
| S4-U10 hump height | ±3 mm | The 20 Nov data (mean error 2.3 mm on a 5 mm ramp) does not meet this yet, so S4-U10 stays failing until tuned |
| S4-U11 tilt | flagged within 200 ms, 10/10 | Confirmation takes 5 × 20 ms = 100 ms |
| SYS-05 telemetry | no gap > 3 s | Same as S1-U12 |

### #48 Start command and Idle/Ready

**Decision.** Add a **START** command over MQTT. The integrated robot waits in
Ready, with the motors off, until START arrives. This keeps the Week 6
system statechart: Initialising → Idle/Ready → (start command) → Navigating.

**Today.** The robot arms on its own once 5 s have passed (`S2_ROBOT_START_MS`),
the IMU is calibrated and S5 has released its start-up HOLD. These become the
conditions for entering Ready.

**Built 21 Nov 2026.**
- S1: `S1_COMMAND_START` = 5, the first reserved code. `tools/mqtt_binary.py
  command start`; the dashboard's Start button now sends START, and
  `all_test` treats START like RESUME.
- VC: `robot_control.c` polls S1 commands each step (non-blocking, at most
  `S1_QUEUE_DEPTH` per step). Once the old auto-arm conditions hold it prints
  `[ROBOT] ready: motors off, waiting for START over MQTT` and arms only after
  START. A START sent during initialisation is kept until Ready. Other
  commands are logged and ignored until #72.
- Demo-day fallback: `S2_AUTO_START` (`robot_control.h`) set to 1 restores
  auto-start. Builds without MQTT default to 1, since nothing could start them.
- Host test: `int/robot_control` (Ready waits, other commands don't start
  it, START arms, early START kept, poll budget).
- Done 21 Nov: robot test PASS (integration TEST_LOG), and the team confirmed
  that the Write-up's "without remote control" wording allows a START
  command (DEVIATIONS P5): START only begins the run; navigation stays
  autonomous.

**Original plan.**
- S1: a START command code in the command queue. The command byte has
  reserved values free (`subsystem1/README.md`).
- VC: a Ready state in `robot_control.c` that arms only on START. This needs
  the VC connected to S1 commands first (#72).
- Host test in `int/robot_control`; then a bench test with the broker.
- Risk to manage: without Wi-Fi the robot never starts. Decide a fallback for
  demo day (for example a build flag that restores auto-start), and check
  that the "without remote control" wording in the Write-up (DEVIATIONS P5)
  allows a start command.

### #54 One distance representation

**Decision.** On the wire, obstacle distance and width are **9-bit cm,
0–400** (IF-05 telemetry). Inside S5, every distance is mm (`UW`), from the
driver (`ultrasonic_measure_mm()`) through scan, profile and plan.

**Why.** Checked in the code: no 16-bit distance appears in S5 or in telemetry.
The only thing S5 sends out is the IF-10 request word, whose 10-bit `value`
holds cm or degrees for a move. The 16-bit field in telemetry is distance
*travelled* (odometry), a different quantity. The VC converts `mm / 10` when
it fills IF-05 (#72). In telemetry, 0 cm can also mean "no echo"; the
`S1_FAULT_NO_ECHO` bit tells them apart.

### #64 Obstacle detection distance

**Decision.** Keep `D_DETECT_MM` = 200 mm (20 cm). NFR5-1 changes to "detect at
20 cm or more and come to rest at least 10 cm away".

**Why.** Every all_test avoidance run since 7 Nov (INT-S5-01) used 20 cm, and
the bypass legs are tuned for it. The measured echo cone is ±20° (S5-U32). At
30 cm it covers about ±11 cm either side of the path, so objects beside the
track could trigger it.

**Conditions.** Buddy 5 adds the original reason for choosing 20 cm to
`testing/subsystem5/TEST_LOG.md`. S5-U40 (#31) must show the robot comes to
rest at least 10 cm from the box. At 20 cm, only 10 cm is left to stop in
after the 2-of-3 confirmation. If S5-U40 fails, restore 30 cm and set
`BYPASS_MIN_GAP_MM` to 150.

### #65 Coarse scan angles

**Decision.** Keep the 50–130° range. FR5-1 changes to a coarse scan at **50,
70, 90, 110 and 130°**.

**Why.** The sensor holder cannot reach 30° or 150°. The servo is clamped to
50–130° in code (S5-U11, S5-U22, since `e7efa12`, 5 Nov). With the measured
±20° echo cone, the 50° and 130° pings still see out to about 30° and 150°.
Less servo travel also makes the scan faster.

**Follow-up.** The S5 test log says the NFR5-2 (coarse ≤ 600 ms) and NFR5-3
(decision ≤ 2 s) limits "are dropped for smoothness". They are **not**
dropped: re-measure both with the sensor mounted (#29).

### #66 Task priorities

**Decision.** Keep the current order: `cyw43` 4, S2 `motion_task` 14, S3 16,
S4 18, S5 20. Week 6 made S5 the highest.

**Why.**
- **Core split.** S2 and S3 are pinned to core 1 and `cyw43` runs on core 0.
  Priority only orders tasks on the same core, so Wi-Fi cannot preempt motor
  control or IR sampling.
- **S4 timing.** S4-U05 at 10 ms overran equally with Wi-Fi on and off (1007
  vs 1010 overruns; `testing/subsystem4/TEST_LOG.md`, 20 Nov). The 12 ms IMU
  cycle was the cause, not Wi-Fi. At 20 ms there were 0 overruns with Wi-Fi
  joined.
- **Safety comes through the VC, not S5's priority.** S5 only requests moves
  (IF-10) and never drives the motors. The VC applies them at S2's priority.
  S5's slow work is servo travel and 60 ms echo cycles, so a lower priority
  costs it little.

**Still to measure.** S1-U17 (#15) and S2-U12 (#19): control-loop lateness with
Wi-Fi on. If it is too late, lower the Wi-Fi priority.

### #67 Tilt

**Decision.** Tilt is **report-only**. S4 still detects and logs it, and S5
still ignores ultrasonic readings while tilted. The VC no longer stops on
`IMU_MOTION_TILT` or `IMU_EVENT_TILT` (`robot_control.c`).

**Why.** It matches the PreTri 1 decision and `TRIBUNAL_STATUS.md`. A tilt stop
latches until reboot, so one false tilt (the 20 Nov ramp log has 22, though
that ramp was under one side and the robot was handled) would end the demo
run. The real danger, the robot being picked up, is covered by the lift
check (#53). IMU faults, stale IMU data and (until #53 removed it on 21 Nov,
replacing it with the lift check) collision still stop the robot.

### #68 Line loss

**Decision.** On line loss the robot **stops and waits**. Statechart 3's
"Search Line" becomes "stopped, waiting for the line".

**Why.** The course has no gaps or dashed sections, so losing the line means
something went wrong, and a blind search can drive off the board. S5 already
does its own bounded line seek after a bypass (`LINE_SEEK_MAX_CM`, 50 cm). A
second search would compete with it. Today: S3 queues LOST after 100 ms; the
VC stops after 200 ms; it re-arms after centre and right have seen black for
1 s with the wheels still. In `integrated`, S3 latches a LINE fault after 2 s
(`LINE_BARCODE_SEARCH_MS`).

# Subsystem 5 — test log

**Subsystem:** Ultrasonic scanning, obstacle profiling & avoidance  ·  **Student ID:** 2500434  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | TIMERAWL counts at 1 MHz under the kernel, including after a cold start (watchdog tick set to 12 cycles by `rp2040_timer_start`) | (enables NFR5-6) | auto (planned) / manual | Manual until automated: 1. Unplug and replug USB (cold start). 2. Read TIMERAWL, `tk_dly_tsk(100)`, read again. 3. Print the difference. | Difference 100 000 – 120 000 µs (100 ms plus up to one 10 ms tick) |
| S5-U02 | Distance accuracy over 5–100 cm | NFR5-6 | manual | 1. Sensor fixed on the bench, aimed at a flat hard target. 2. Measure the true distance at 5, 10, 20, 50 and 100 cm. 3. Record 10 readings at each distance. | Every reading within ±20 mm of the true distance |
| S5-U03 | Validity filter: no echo, < 2 cm, > 400 cm and echo-still-high are reported invalid | NFR5-5 | auto (planned) | Needs the status/distance logic of `ultrasonic_measure_mm` split into a pure function first; then feed fixed echo times in `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Servo calibration: robot angle maps to the correct physical direction (90 = forward, > 90 = left) | FR5-1 | manual | 1. Fold an A4 corner to mark 45° each side of forward. 2. Command 45° and 135°. 3. Check the pointer on the horn against the marks. 4. Check that 120° and 150° turn left. | Pointer on each 45° mark; > 90° turns left |
| S5-U05 | Coarse scan duration (5 angles: 90, 60, 30, 120, 150; one ping each) | NFR5-2, FR5-1 | manual | 1. Flash the normal image. 2. Read the `coarse scan N ms` line printed after each scan. 3. Observe at least 5 scans. | Every scan ≤ 600 ms |
| S5-U06 | Detection → chosen avoidance action duration | NFR5-3 | auto (planned) | Time from detection to the decision with `rp2040_time_us()` | ≤ 2000 ms |
| S5-U07 | Profile of a known box (width, centre offset, closest point) | FR5-3 | manual (planned) | Box of known size at a measured position; compare the printed profile | Within the tolerance stated in the result notes |
| S5-U08 | Planner picks left / right / reverse / NO_PATH correctly | FR5-4 | auto (planned) | Fixed scan data into the pure planner function | Expected decision for each case |
| S5-U09 | Packed message round trip; out-of-range and reserved values rejected | CR1 | auto (planned) | Pack, then unpack, fixed values | All equal or rejected |
| S5-U10 | RAM and message size before/after minimum-bit packing | CR1 | manual (planned) | `arm-none-eabi-size` on the `.elf`, plus the `.map` | Numbers recorded |
| S5-U11 | `servo_angle_to_pulse_us`: calibration maths and clamping | FR5-1 | auto (planned) | 30/60/90/120/150 and out-of-range angles in `subsystem5_test.c` | 850/1190/1530/1870/2210 µs; < 20° or > 160° clamped |
| S5-U12 | `servo_settle_ms`: symmetric, 0 for no move, rounds up, includes margin | NFR5-2 | auto (planned) | Fixed angle pairs in `subsystem5_test.c` | (90,90) = 0; (60,30) = (30,60) = 70; (30,120) = 170 |
| S5-U13 | `obstacle_filter_update` (2-of-3 filter): one close reading does not confirm; two of the last three do; only `ULTRASONIC_OK` readings under `D_DETECT_MM` count as close; `TOO_NEAR` and `BUSY` leave the history unchanged | NFR5-1, NFR5-5 | auto (planned) | Feed fixed sequences into `obstacle_filter_update` in `subsystem5_test.c`, starting from history 0: (a) OK 250 → FALSE; (b) OK 250, OK 250 → TRUE; (c) OK 250, OK 500, OK 250 → TRUE; (d) OK 250, NO_ECHO, NO_ECHO → FALSE; (e) OK 250, TOO_NEAR, TOO_NEAR → FALSE (history still 1); (f) NO_ECHO ×3 and TOO_FAR ×3 → FALSE; (g) OK 300 ×2 → FALSE (boundary: 300 is not < 300); (h) `p_history` = NULL → FALSE | `[TEST] S5-U13 PASS` |
| S5-U14 | Monitoring confirms a real obstacle ahead and ignores a brief one | NFR5-1, NFR5-5 | manual | 1. Sensor fixed facing forward (servo at 90°) with ≥ 1 m of open space in front. 2. Flash the normal image; watch the console for 10 s. 3. Wave a hand quickly through the beam at ~20 cm, 5 times. 4. Push a flat box slowly towards the sensor from 60 cm to 20 cm. 5. Repeat step 4 three times. | Step 2: only `S5: MONITORING`. Step 3: no `OBSTACLE` line (a slow wave may trigger; note it). Steps 4–5: each time one `S5: OBSTACLE at N mm` with N between 250 and 300, then one scan printout, then `S5: MONITORING` again after 3 s |
| S5-U15 | `fine_scan_window`: window = first hit − 15° to last hit + 15°, limited to 30..150°; cut to closest hit ± 35° when it would exceed 15 points (NFR5-3 budget). A hit is `ULTRASONIC_OK` under `D_PROFILE_MM` (600 mm) | FR5-2, NFR5-3 | auto (planned) | Fixed 5-point coarse scans into `fine_scan_window` in `subsystem5_test.c`: (a) hit at 90 only → 75..105; (b) hit at 30 only → 30..45; (c) hit at 150 only → 135..150; (d) hits at 60 and 90 → 45..105 (13 points, no cut); (e) hits at 60 and 120, closest at 120 → 19 points → cut to 85..150 (clamped from 155); (f) hits at 30, 90 and 120 with 90 closest → 30..135 is 22 points (the 2026-11-01 bench case) → cut to 55..125; (g) no hits (all NO_ECHO, or all OK ≥ 600 mm) → FALSE; (h) a 700 mm OK reading is not a hit; (i) any NULL pointer → FALSE | `[TEST] S5-U15 PASS` |
| S5-U16 | `scan_median3`: 3-point median across neighbouring angles; only where all three are `ULTRASONIC_OK`; end points and invalid points copied unchanged | FR5-2, NFR5-5 | auto (planned) | Fixed arrays into `scan_median3`: (a) 400, 1830, 400 → middle 400 (spike removed); (b) 400, 400, 1500 → unchanged (edge kept sharp); (c) 400, NO_ECHO, 400 → middle unchanged (still NO_ECHO); (d) OK 400, OK 1830, NO_ECHO → middle unchanged (neighbour invalid); (e) first and last points never changed; (f) count 0, 1 and 2 → no crash, output equals input; (g) NULL pointer → returns without writing | `[TEST] S5-U16 PASS` |
| S5-U17 | Fine scan locates a real obstacle (needs the sensor mounted on the horn) | FR5-2 | manual | 1. Sensor on the horn. 2. Box about 25 cm ahead and slightly left (about 100–110°). 3. Let monitoring trigger. 4. Repeat 3 times. | Window covers the box's angles; smoothed (`med`) distances at the box's angles agree within 20 mm across the 3 runs; distances outside the box are clearly larger |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
*(planned)* = defined but not yet runnable.

## Results

Add a new row for every run. Never edit or delete old rows.

Rows dated before the first subsystem 5 commit were run on uncommitted code
during bring-up, and their console output was not saved. They are recorded
for the history of the fixes; re-run these tests on a commit with saved logs
before relying on them.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S5-U01 | uncommitted (before `5640215`) | 2500434 | FAIL | console, not saved | Cold start: 2 579–2 581 µs per ~110 ms (counts ~42.7× too slowly). Watchdog TICK divider resets to 0 (= 512 cycles) at power-up; it only looked right straight after flashing because the boot ROM had set it to 12. Fix: write ENABLE \| 12 to WATCHDOG_TICK at startup |
| 2026-10-27 | S5-U01 | uncommitted (before `5640215`) | 2500434 | PASS | console, not saved | After the fix: 109 969 and 109 975 µs per `tk_dly_tsk(100)` |
| 2026-10-27 | S5-U02 | uncommitted (before `5640215`) | 2500434 | FAIL | console, not saved | Software-timed echo (polling ECHO with TIMERAWL), hand-held, A4 reference 297 mm: 270, 293, 303, 309 mm. Spread 39 mm > ±20 mm; pre-emption during the echo shifts the start or end time. Fix: time the echo in hardware (PWM slice, level-gated), done in `5640215` |
| 2026-10-28 | S5-U04 | uncommitted (before `ff5483c`) | 2500434 | PASS | visual check, not recorded | Calibration: 45° R = 1020 µs, forward = 1530 µs, 45° L = 2040 µs (midpoint of the 45° points equals the measured forward: linear). Horn re-seated first, since 1500 µs had pointed 90° left of forward. Servo tested with the sensor NOT mounted; re-check direction with the sensor on the robot |
| 2026-10-28 | S5-U05 | uncommitted (between `a6c9b18` and `b15bbc5`) | 2500434 | FAIL | console, not saved | 650 ms. Every `tk_dly_tsk` adds about one 10 ms tick, and the scan makes 10 of them. Fix: no settle wait when the servo does not move; 20 ms scan listening window instead of 30 ms (range ~3 m) |
| 2026-10-28 | S5-U05 | `b15bbc5` | 2500434 | PASS | console, not saved | 570 ms. Readings steady from scan to scan. Servo tested without the sensor mounted: re-check timing and steadiness under load (settle margin may need increasing) |
| 2026-11-01 | S5-U06 | uncommitted (after `6d54665`) | 2500434 | FAIL | console screenshot, not saved to `logs/` | Coarse + fine scan only (profiling and planning not written yet): 2299 ms > 2000 ms. Window 30..135° = 22 points; (2299 − 570) / 22 ≈ 79 ms per fine point (two tick-rounded sleeps each: settle + echo window). Fix: cap the fine window at 15 points, centred on the closest coarse hit when wider (`FINE_BUDGET_POINTS`); expected worst case ≈ 1.8 s. Sensor was fixed (not on the horn), but the servo made every move, so the timing is valid |
| 2026-11-01 | S5-U16 | uncommitted (after `6d54665`) | 2500434 | PASS | console screenshot, not saved to `logs/` | Checked by hand from the bench printout (auto test still planned): every `med` equals the median of its raw neighbours; spike at 105° (187, 497, 222) → 222; ambiguous zig-zag 187, 497, 222, 566 became one clean step 187, 222, 497, 566; end points 30° and 135° unchanged. Sensor was fixed, so the values vary with time, not angle: this checks the filter maths only, not obstacle shape (see S5-U17) |

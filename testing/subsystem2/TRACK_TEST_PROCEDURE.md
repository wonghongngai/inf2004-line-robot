# S2 track test of the promoted values (S2-U08, U09, U12, INT-08)

Checks PR #105's values on the integrated build: turn constants 24.2 (right
pivot) / 25.0 (left pivot), straight PID 6.0 / 0.5 / 0. Both were tuned at
25 % PWM on the floor; track pivots run at 12.5 / 15 %.

## Build (user's shell, Pico SDK set up)
    sh build.sh integrated
-> `build_make/robot-integrated.uf2`. Commit first and record `git rev-parse --short HEAD`.
Hold BOOTSEL, plug in, copy the .uf2. Keep the robot still 1 s after boot (IMU baseline).
Send the letter map after every reboot (README 3.2). Press Start on the dashboard.

## Log
    powershell -ExecutionPolicy Bypass -File testing\subsystem5\serial_monitor.ps1 -LogDir testing\subsystem2\logs
Start it before the reset. One boot per test, so each log is one run.

## S2-U12 (do first, also covers U08/U09 drives): 5 min integrated run
Read the `ctrl late=N/M gap_max=.. loop_max=.. ms` line each second.
Pass: late=0 at the end; record the largest gap_max / loop_max.

## S2-U09: 10 x left 90, 10 x right 90, 10 x U-turn 180
Barcode map: left/right/uturn letters on a track with a junction after each
barcode. Per turn record: angle by eye or protractor on the floor
(+/-15 deg), and whether the line is reacquired (FOLLOW resumes, no FAULT).
Note the PWM: pivots run at 12.5 % (right) / 15 % (left). If turns are short
at this speed, raise `S2_LEFT_TURN_TICKS_PER_DEGREE` (right pivot) /
`S2_RIGHT_TURN_TICKS_PER_DEGREE` (left pivot) by the error ratio; if long,
lower them. The U-turn uses both constants.

## S2-U08: drives straight, 2 m, drift <= 10 cm (decided: straight tape, integrated build)
Decision: the integrated build stops on line-lost, so the test runs on 2 m of
straight tape with the line-follow trim and the PID both active. The tape
corrects drift, so this checks the stacked correction, not the bare PID.
Setup: 2 m straight tape, robot centred on it at the start. Mark the end.
Measure: the sideways distance from the tape centre to the robot centre at
the end point, and the distance travelled from the log. Repeat 3 times.
Pass: lateral offset <= 10 cm. Also read the `[MOTION]` lines for
oscillation (left/right PWM alternating by more than a few points per
100 ms) or the trim pinned at S2_FOLLOW_STRAIGHT_MAX_TRIM.
Record in the row that the line was present (not a no-line drift test).

## INT-08 (#33): 4 commands x 5 at a junction, 20/20 correct
Same track and map as U09; count correct turns.

## Rollback
subsystem2.h: turn 20.3556 / 22.9074, PID 0 / 0 / 0.

## Recording
Trim the raw `*_console.txt` (gitignored) to the runs, save as
`testing/subsystem2/logs/2026-11-DD_S2-U12.txt` etc., add Results rows with
commit, never edit old rows, record every FAIL.

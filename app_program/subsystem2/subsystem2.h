/*
 * subsystem2.h
 * Subsystem 2 -- Buddy 2 Motion Control.  Student ID: 2503986.
 *
 * Public interface.  Other subsystems should request movement through
 * subsystem2_send_command(); they must never write GP8..GP11 directly.
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_control.h"
#include "subsystem2_hw_test.h"

/* TEAM.md allocates subsystem 2 priority band 14-15. */
#define S2_PRI_MOTION          14

/* One Motion Control task, as per the subsystem architecture.
   It samples encoder GPIO every 1 ms and runs PID/state control every 10 ms.
   The 10 ms value is a DEVELOPMENT value and must be validated on hardware. */
#define S2_ENCODER_POLL_MS      1U
#define S2_CONTROL_PERIOD_MS   10U

/* Up to four movement requests may wait.  No data compaction is used. */
#define S2_COMMAND_QUEUE_DEPTH  4U

/* ---------------------------------------------------------------------
 * CALIBRATION
 * ---------------------------------------------------------------------
 * Physical measurements supplied on 2026-10-29; see testing/subsystem2/CALIBRATION.md.
 * Distance uses the three measured 100 cm runs, not wheel circumference.
 * Turn scales belong to the MOVING wheel: left wheel for a right pivot,
 * right wheel for a left pivot. Revalidate after encoder/sampling changes.
 */
#define S2_CALIBRATION_READY                 1
// #define S2_LEFT_TICKS_PER_CM                 103.5267f
// #define S2_RIGHT_TICKS_PER_CM                118.5634f
#define S2_LEFT_TICKS_PER_CM   23.10f //updated calibration value
#define S2_RIGHT_TICKS_PER_CM  18.83f
/* 22 Nov (S2-U07): from hand-pivoted 360-degree turns, 8287 / 360 and 8343 / 360.
   Was 20.3556 (right turn) and 22.9074 (left turn), from 29 Oct 90-degree pivots. */
#define S2_LEFT_TURN_TICKS_PER_DEGREE         24.2f     /* right turn, driven: see CALIBRATION.md */
#define S2_RIGHT_TURN_TICKS_PER_DEGREE        25.0f     /* left turn, driven: see CALIBRATION.md */
#define S2_BASE_PWM_PERCENT                  55.0f
#define S2_MIN_EFFECTIVE_PWM_PERCENT         15.0f    /* shared floor covers right motor */
#define S2_MAX_PWM_PERCENT                   100.0f
#define S2_DISTANCE_TOLERANCE_CM             1.0f
#define S2_ANGLE_TOLERANCE_DEG               2.0f
#define S2_ENCODER_STALL_TIMEOUT_MS          500U

/* Straight-line PID (left-minus-right travel, in cm, to a PWM trim). Tuned 22 Nov
   (S2-U10) on the floor at 25 % PWM: 0 / 0 / 0 drifted 94-126 mm over 100 cm,
   6.0 / 0.5 / 0 held it to 3 mm. Not yet checked at the 12.5 / 15 % track speeds. */
#define S2_PID_KP                            6.0f
#define S2_PID_KI                            0.5f
#define S2_PID_KD                            0.0f
#define S2_PID_INTEGRAL_LIMIT                100.0f
#define S2_PID_CORRECTION_LIMIT              30.0f

/* #8 line-following motor numbers, tuned on the track in all_test and used
   by BOTH builds (all_test's ALL_CRUISE_* / ALL_LINE_* read these), so they
   cannot drift apart again. Following uses no moving floor and no stall
   stop, as all_test always did. */
#define S2_FOLLOW_LEFT_PWM_PERCENT           15.0f  /* was 12.5, raised 22 Nov for the S2-U08 track test */
#define S2_FOLLOW_RIGHT_PWM_PERCENT          19.0f  /* right motor is weaker; was 15.0, 18.0 drifted right 22 Nov */
#define S2_FOLLOW_BOOST_PWM_PERCENT          4.0f   /* outer wheel while steering */
#define S2_FOLLOW_INNER_PWM_PERCENT          5.0f   /* inner wheel, soft correction */
#define S2_FOLLOW_HARD_INNER_PWM_PERCENT     0.0f   /* inner wheel, hard correction */
#define S2_FOLLOW_RAMP_PWM_PER_S             80.0f  /* speed-ups only */

/* Straight stretches (C/R both white) are held straight by the encoders, as
   all_test's white_straight_step(): the wheel that is behind since the
   stretch began gets a boost (never a cut) of KP per cm, capped, ramped. */
#define S2_FOLLOW_STRAIGHT_KP                2.0f   /* PWM points per cm */
#define S2_FOLLOW_STRAIGHT_MAX_TRIM          2.5f
#define S2_FOLLOW_STRAIGHT_TRIM_RATE         5.0f   /* PWM points per second */
#define S2_FOLLOW_STRAIGHT_DEADBAND_CM       0.10f
/* I term (23 Nov, #23: the robot drifted one way). The P trim above restarts
   at 0 on every stretch, so on short stretches between corrections it never
   builds up and a steady motor mismatch keeps pushing the same way. This
   bias learns that mismatch from the same wheel difference, KI PWM points
   per cm per second, and is kept across stretches, turns and stops (only
   motion_control_init clears it). Added to the P trim on straights only.
   0 = off. The steer log line prints it as `bias=` in tenths of a point. */
#define S2_FOLLOW_STRAIGHT_KI                1.0f
#define S2_FOLLOW_STRAIGHT_MAX_BIAS          3.0f
/* While GP28 is on a bar or a barcode capture is open, line following runs
   at this fraction of its PWM, and a left correction keeps the left
   (scanner-side) wheel at the scan inner PWM instead of pivoting on it. */
#define S2_FOLLOW_SCAN_SCALE                 0.9f
#define S2_FOLLOW_SCAN_LEFT_INNER_PWM_PERCENT 8.0f

/* Interrupt-encoder calibration, from three 100 cm hand pushes (2026-11-06;
   right trimmed 2026-11-08 for drift). S2_*_TICKS_PER_CM above predate the
   interrupt encoder; all_test's S5 moves, odometry and height use these.
   Left 117.8 -> 116.2 on 23 Nov (#23): three ~210 cm bare-floor runs with
   the encoder trim holding the wheels level (right 0.7-1.2 cm ahead by the
   encoders) each veered ~20 cm right, so the left wheel goes ~1.4 % further
   per tick than 117.8 says (wheel centres ~10.5 cm apart: 8 cm inner gap
   plus the tyre width, estimated). */
#define S2_ENC_LEFT_TICKS_PER_CM             116.2f
#define S2_ENC_RIGHT_TICKS_PER_CM            118.3f

/* all_test's drive numbers for distance moves and pivots. Both builds read
   these. Moves ramp 100 PWM points/s each way; a stopped wheel stops at once. */
#define S2_MOVE_LEFT_PWM_PERCENT             12.5f
#define S2_MOVE_RIGHT_PWM_PERCENT            15.0f
#define S2_MOVE_RAMP_PWM_PER_S               100.0f
/* Junction pivots (S2 turn calibration): right pivot drives the left wheel,
   left pivot drives the right wheel. The U-turn counter-rotates the wheels
   and counts half the right wheel's pivot arc. */
#define S2_JUNCTION_RIGHT_PIVOT_PWM_PERCENT  18.0f  /* was 12.5: near stalling on the track, 22 Nov; left pivot is 20 */
#define S2_JUNCTION_LEFT_PIVOT_PWM_PERCENT   20.0f  /* was 15.0: right wheel stalled mid left turn on the track, 22 Nov */
#define S2_UTURN_LEFT_PWM_PERCENT            (-18.0f)  /* was -12.5; right wheel stalled at 15, 22 Nov */
#define S2_UTURN_RIGHT_PWM_PERCENT           20.0f  /* was 15.0 */
/* S5 avoidance: forward legs hold the heading by cutting the leading wheel
   by KP per cm ahead; pivots drive one wheel at 20 % around the other, on
   an effective radius of S2_AVOID_PIVOT_TRACK_CM. */
#define S2_AVOID_HEADING_KP                  15.0f  /* PWM points per cm */
#define S2_AVOID_PIVOT_PWM_PERCENT           20.0f
#define S2_AVOID_PIVOT_TRACK_CM              11.25f
#define S2_AVOID_TURN_TICKS_PER_DEGREE(ticks_per_cm) \
    (3.14159265f / 180.0f * S2_AVOID_PIVOT_TRACK_CM * (ticks_per_cm))

/* Receiver-owned command message buffer: Vehicle Controller -> Buddy 2. */
IMPORT ID mbf_motion_cmd;
IMPORT ID flg_motion_stop;

/* Public integration interface. */
ER subsystem2_send_command(const MOTION_COMMAND *command);
ER subsystem2_get_status(MOTION_STATUS *status);

/* Private to subsystem 2, declared here for task creation/tests. */
IMPORT ID mtx_motion_status;
IMPORT void motion_task(INT stacd, void *exinf);
IMPORT void subsystem2_publish_status(const MOTION_STATUS *status);

#endif /* SUBSYSTEM2_H */

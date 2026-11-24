#ifndef S2_ROBOT_CONTROL_H
#define S2_ROBOT_CONTROL_H

#include "motion_control.h"

/* Enter measured black/white GPIO masks before enabling integrated motion.
   Left=bit2, centre=bit1, right=bit0. No assumed sensor polarity. */
#ifndef S2_LINE_CALIBRATION_READY
#define S2_LINE_CALIBRATION_READY 1
#endif
#define S2_LINE_BLACK_RAW 7u
#define S2_LINE_WHITE_RAW 0u
#define S2_LINE_LAYOUT    LINE_BARCODE_LAYOUT_SIDE
#define S2_ROBOT_START_MS 5000u
/* #48 (decided 20 Nov 2026): after S2_ROBOT_START_MS, a calibrated IMU and
   S5's start-up hold, the robot waits in Ready, motors off, until an MQTT
   START command (S1_COMMAND_START) arrives. 1 restores the old auto-start:
   the demo-day fallback if Wi-Fi is unavailable. Builds without MQTT always
   auto-start, since nothing could ever send START to them. */
#ifndef S2_AUTO_START
#if defined(TM_MQTT) && TM_MQTT
#define S2_AUTO_START 0
#else
#define S2_AUTO_START 1
#endif
#endif
#define S2_IMU_STALE_MS          100u
#define S2_LINE_UNSAFE_MS        200u
#define S2_LINE_REARM_MS         1000u
#define S2_ROBOT_TURN_TIMEOUT_MS 8000u /* all_test's junction movement limit */
/* S5 avoidance (all_test): HOLD is answered once the robot has had this long
   to stop, and each leg or pivot may take this long (an 80 cm leg at
   12.5/15 % takes about 10 s). */
#define S2_AVOID_HOLD_SETTLE_MS  150u
#define S2_AVOID_MOVE_TIMEOUT_MS 20000u
#define S2_DEMO_TIMEOUT_MS       15000u

/* #8 (21 Nov): line model ported from all_test's tuned white-track mode.
   On this robot the centre (GP27) and right (GP0) sensors sit either side
   of the black line: C/R = 0/0 on the line (straight), 0/1 drifting left
   (steer right), 1/0 drifting right (steer left), 1/1 a junction.
   0 restores the S3 position model (line under both sensors). */
#ifndef S2_LINE_STRADDLE
#define S2_LINE_STRADDLE 1
#endif
#define S2_LINE_CONFIRM_MS 5u  /* a left<->right reversal must persist */
#define S2_WHITE_CLEAR_MS  30u /* hold a correction this long after white */
#define S2_LINE_HARD_MS \
    150u /* soft correction becomes hard (inner wheel stops) */
/* Lost line in the straddle model (#23). Both white means centred or lost.
   After a correction of at least S2_LINE_LOST_ARM_MS clears to white, the
   robot is angled back across the tape, so C or R should see black again
   soon. No black within S2_LINE_LOST_MM of travel (encoders): probe (below),
   and if the probe finds no black either, the line is lost: stop, and
   re-arm only once C or R has seen black (or PAUSE + RESUME).
   S2_LINE_LOST_MM is a first guess: tune it from the `watch max=` figure in
   the once-a-second `steer` log (S3-U31). 0 turns the check off. */
#ifndef S2_LINE_LOST_MM
#define S2_LINE_LOST_MM 150u
#endif
/* Before stopping, steer softly (this follow value) toward the side the
   tape was last under for up to S2_LINE_PROBE_MM more; any black ends it
   and following carries on (#23: two false stops on 23 Nov had the tape
   still between C and R). 0 = stop at S2_LINE_LOST_MM, no probe. */
#ifndef S2_LINE_PROBE_MM
#define S2_LINE_PROBE_MM 100u
#endif
#define S2_LINE_PROBE_STEER 0.5f
#define S2_LINE_LOST_ARM_MS S2_LINE_HARD_MS
#define S2_LINE_LOST_GAP_MS \
    50u /* a longer gap in line following drops the watch */
/* Drive onto the crossing before pivoting. all_test's "5 cm" counts 5 x
   S2_RIGHT_TICKS_PER_CM = 94 right-wheel ticks, which is 0.8 cm with the
   interrupt encoder: keep that tuned distance. */
/* 22 Nov (S2-U09). Pivots swing the sensors about the stationary wheel, and
   the right IR sensor sits about 2 cm right of the centre one, so the line is
   held about 1 cm right of the robot's centreline. That puts the sensors
   about 2 cm further along after a left pivot than after a right pivot, so
   the two turns get separate advances. Left: 0.8 cm all_test distance plus
   3.5 cm (+2.0 closed only 1 of 2 cm short). Right: 2.0 cm more, from the
   right turn ending about 1 cm short at the left's 4.3 cm. */
#define S2_TURN_ADVANCE_BASE_CM \
    (5.0f * S2_RIGHT_TICKS_PER_CM / S2_ENC_RIGHT_TICKS_PER_CM)
#define S2_TURN_ADVANCE_LEFT_CM \
    (3.7f +                     \
     S2_TURN_ADVANCE_BASE_CM) /* 4.5 cm: 4.3 missed on both sides, 5.0 overshot */
#define S2_TURN_ADVANCE_RIGHT_CM        (5.5f + S2_TURN_ADVANCE_BASE_CM)
/* U-turn and straight-on keep the left value */
#define S2_TURN_ADVANCE_CM              S2_TURN_ADVANCE_LEFT_CM
/* A left/right pivot stops on the line once past this fraction of its arc and
   the centre sensor is black (not before: the sensors start on the crossing).
   22 Nov: 0.6 stopped left pivots at 74-80 % of the arc with the robot 15-25
   degrees short of the line; right pivots stopped at 91-99 %. */
#define S2_PIVOT_LINE_MIN_FRACTION      0.9f
/* Left pivot only: 0.9 overshot the line, so it may stop on the line from 80 %. */
#define S2_PIVOT_LINE_MIN_FRACTION_LEFT 0.8f
#define S2_JUNCTION_SETTLE_MS 500u /* ignore the crossing just handled */

/* Called only by the motor-owning task; no second writer to motor GPIO. */
void robot_control_step(MOTION_CONTROLLER * controller, uint32_t now,
                        int32_t left, int32_t right);

#endif

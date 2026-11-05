/**
 * @file    drv_servo.h
 * @brief   SG90 scanning-servo driver.  A PWM slice outputs a 50 Hz signal;
 *          the pulse width (about 0.5-2.5 ms) sets the horn's angle.
 *
 * Angles are in the ROBOT's frame: 90 = straight ahead, > 90 = left,
 * < 90 = right. Only obstacle_task may call these functions.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <tk/tkernel.h> /* UW */

#define SERVO_ANGLE_CENTRE_DEG (90U) /* straight ahead (after calibration) */

/* safety limits: requests outside this range are clamped */
#define SERVO_ANGLE_MIN_DEG (20U)
#define SERVO_ANGLE_MAX_DEG (160U)

/* ---- Smooth (gliding) motion ------------------------------------------
 * Angles in tenths of a degree ("ddeg", 900 = forward).  The position is
 * worked out from the TIME since the move started (1 MHz timer), not by
 * counting loop passes, so a late task (kernel tick rounding, higher
 * priority tasks) can't make the horn stall.  The PWM latches a new pulse
 * only at the start of each 20 ms period, so polling faster than that
 * gives the servo evenly spaced positions.
 *
 * Speeds are chosen so every 20 ms pulse moves the horn 1.2-1.6 deg
 * (14-18 us), above the SG90's 10 us dead band: smaller changes are
 * ignored until they add up, then the horn jumps (that was the jitter).
 * Speed ramps START -> CRUISE over RAMP_MS and back at the end.
 * CRUISE also keeps 5 deg pings >= 62 ms apart (sensor needs >= 60 ms).
 * [Tune on the robot] */
#define SERVO_DDEG_PER_DEG        (10U)
#define SERVO_GLIDE_POLL_MS       (5U)   /* several updates per PWM period */
#define SERVO_GLIDE_START_DDEG_S  (600U) /* 60 deg/s: 1.2 deg per pulse    */
#define SERVO_GLIDE_CRUISE_DDEG_S (800U) /* 80 deg/s: 1.6 deg per pulse    */
#define SERVO_GLIDE_RAMP_MS       (100U) /* speed-up / slow-down time      */
#define SERVO_GLIDE_SETTLE_MS     (60U)  /* after arriving, before a ping  */

/** Set up the PWM slice and point the servo straight ahead. Call once. */
IMPORT void servo_init(void);

/** Point the servo at 'angle_ddeg' at once (clamped).  Returns at once. */
IMPORT void servo_set_angle_ddeg(UW angle_ddeg);

/** Last commanded angle, in ddeg. */
IMPORT UW servo_get_angle_ddeg(void);

/** Pure: angle in ddeg -> pulse width in us, clamped. */
IMPORT UW servo_ddeg_to_pulse_us(UW angle_ddeg);

/**
 * Pure: how long a glide over 'distance_ddeg' takes, in ms.
 * Moves of at least 14 deg ramp START -> CRUISE -> START; shorter ones run
 * at START speed throughout.
 */
IMPORT UW servo_profile_total_ms(UW distance_ddeg);

/**
 * Pure: how far a glide over 'distance_ddeg' has got after 'elapsed_ms',
 * in ddeg (rounded).  0 at the start, rises without ever going back,
 * equals distance_ddeg from servo_profile_total_ms() onwards.
 */
IMPORT UW servo_profile_ddeg(UW distance_ddeg, UW elapsed_ms);

/** Start a glide from the current angle to 'angle_deg' (clamped).
 *  Returns at once; then call servo_glide_update() often. */
IMPORT void servo_glide_begin(UW angle_deg);

/**
 * Output the position the current glide should be at now.  Call at least
 * every SERVO_GLIDE_POLL_MS (more often is fine, late is tolerated).
 * @return TRUE once the target is reached (or no glide is running).
 */
IMPORT BOOL servo_glide_update(void);

/** Glide to 'angle_deg' (blocking), then wait SERVO_GLIDE_SETTLE_MS. */
IMPORT void servo_glide_to(UW angle_deg);

/** Point the servo at 'angle_deg' (clamped).  Returns immediately: the
 *  horn needs time to get there, so the caller must wait before pinging. */
IMPORT void servo_set_angle(UW angle_deg);

/** Pure (no hardware): angle -> pulse width in us, clamped. Public so
 *  the on-target tests can check the calibration maths. */
IMPORT UW servo_angle_to_pulse_us(UW angle_deg);

/** Move to 'angle_deg' (clamped) and SLEEP until it has arrived.  Now a
 *  smooth glide (servo_glide_to); kept so callers need not change. */
IMPORT void servo_move_and_settle(UW angle_deg);

/** JUMP to 'angle_deg' (clamped) at the servo's full speed, then SLEEP
 *  servo_settle_ms() so the horn has stopped before a ping.  Used by the
 *  coarse scan, which only needs the 5 angles, not the path between. */
IMPORT void servo_jump_to(UW angle_deg);

/** Pure: how long a move from 'from_deg' to 'to_deg' needs, in ms. */
IMPORT UW servo_settle_ms(UW from_deg, UW to_deg);

#endif /* DRV_SERVO_H */
/**
 * @file    drv_servo.h
 * @brief   SG90 scanning-servo driver.  A PWM slice outputs a 50 Hz signal;
 *          the pulse width (about 0.5-2.5 ms) sets the horn's angle.
 *
 * Angles are in the ROBOT's frame: 90 = straight ahead, > 90 = left,
 * < 90 = right.  Only obstacle_task may call these functions.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <tk/tkernel.h> /* UW */

#define SERVO_ANGLE_CENTRE_DEG (90U)  /* straight ahead (after calibration) */
#define SERVO_ANGLE_MIN_DEG    (20U)  /* safety limits: requests outside   */
#define SERVO_ANGLE_MAX_DEG    (160U) /* this range are clamped            */

/** Set up the PWM slice and point the servo straight ahead.  Call once. */
IMPORT void servo_init(void);

/** Point the servo at 'angle_deg' (clamped).  Returns immediately: the
 *  horn needs time to get there, so the caller must wait before pinging. */
IMPORT void servo_set_angle(UW angle_deg);

/** Pure (no hardware): angle -> pulse width in us, clamped.  Public so
 *  the on-target tests can check the calibration maths. */
IMPORT UW servo_angle_to_pulse_us(UW angle_deg);

/** Point the servo at 'angle_deg' (clamped) and SLEEP until it has
 *  arrived.  Blocks the caller for servo_settle_ms(). */
IMPORT void servo_move_and_settle(UW angle_deg);

/** Pure: how long a move from 'from_deg' to 'to_deg' needs, in ms. */
IMPORT UW servo_settle_ms(UW from_deg, UW to_deg);

#endif /* DRV_SERVO_H */
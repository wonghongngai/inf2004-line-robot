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
#include "legacy_example.h"

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
 * Physical measurements supplied on 2026-10-29; see CALIBRATION.md.
 * Distance uses the three measured 100 cm runs, not wheel circumference.
 * Turn scales belong to the MOVING wheel: left wheel for a right pivot,
 * right wheel for a left pivot. Revalidate after encoder/sampling changes.
 */
#define S2_CALIBRATION_READY                 1
#define S2_LEFT_TICKS_PER_CM                 103.5267f
#define S2_RIGHT_TICKS_PER_CM                118.5634f
#define S2_LEFT_TURN_TICKS_PER_DEGREE         20.3556f  /* right turn: 1832 / 90 */
#define S2_RIGHT_TURN_TICKS_PER_DEGREE        22.9074f  /* left turn: 2061.6667 / 90 */
#define S2_BASE_PWM_PERCENT                  55.0f
#define S2_MIN_EFFECTIVE_PWM_PERCENT         15.0f    /* shared floor covers right motor */
#define S2_MAX_PWM_PERCENT                   100.0f
#define S2_DISTANCE_TOLERANCE_CM             1.0f
#define S2_ANGLE_TOLERANCE_DEG               2.0f
#define S2_ENCODER_STALL_TIMEOUT_MS          500U

/* PID gains are deliberately not guessed.  Tune these on the real vehicle.
   KP stays 0 until the left encoder's counting direction is confirmed. */
#define S2_PID_KP                            0.0f
#define S2_PID_KI                            0.0f
#define S2_PID_KD                            0.0f
#define S2_PID_INTEGRAL_LIMIT                100.0f
#define S2_PID_CORRECTION_LIMIT              30.0f

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

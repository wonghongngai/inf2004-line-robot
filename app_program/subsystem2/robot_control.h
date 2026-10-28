#ifndef S2_ROBOT_CONTROL_H
#define S2_ROBOT_CONTROL_H

#include "motion_control.h"

/* Enter measured black/white GPIO masks before enabling integrated motion.
   Left=bit2, centre=bit1, right=bit0. No assumed sensor polarity. */
#ifndef S2_LINE_CALIBRATION_READY
#define S2_LINE_CALIBRATION_READY 0
#endif
#define S2_LINE_BLACK_RAW 7u
#define S2_LINE_WHITE_RAW 0u
#define S2_LINE_LAYOUT LINE_BARCODE_LAYOUT_SIDE
#define S2_ROBOT_START_MS 5000u
#define S2_IMU_STALE_MS 100u
#define S2_ROBOT_TURN_TIMEOUT_MS 10000u
#define S2_DEMO_TIMEOUT_MS 15000u

/* Called only by the motor-owning task; no second writer to motor GPIO. */
void robot_control_step(MOTION_CONTROLLER *controller, uint32_t now,
                        int32_t left, int32_t right);

#endif

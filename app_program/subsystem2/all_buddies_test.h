#ifndef ALL_BUDDIES_TEST_H
#define ALL_BUDDIES_TEST_H
#include "../app.h"
#include "../subsystem3/line_barcode.h"
#ifndef ALL_TEST_LINE_LAYOUT
/* GP7=left, GP27=centre, GP0=right, all mounted over the track. */
#define ALL_TEST_LINE_LAYOUT LINE_BARCODE_LAYOUT_TRACK
#endif
#ifndef ALL_TEST_DURATION_MS
/* Zero runs continuously until manually paused. */
#define ALL_TEST_DURATION_MS 0u
#endif
#ifndef ALL_TEST_CONTINUE_ON_SENSOR
/* Sensor detections/failures are diagnostic only in this hardware test. */
#define ALL_TEST_CONTINUE_ON_SENSOR 1
#endif
#define ALL_TEST_PWM_RAMP_PER_SECOND 100.0f /* 0 -> 30% in about 300 ms */
#ifndef ALL_TEST_STOP_ON_COLLISION
/* Requested test behaviour: report IMU collisions but keep following. */
#define ALL_TEST_STOP_ON_COLLISION 0
#endif
/* Combined hardware test; fixed PWM, no calibrated distance/angle motion. */
ER all_buddies_test_start(void);
#endif

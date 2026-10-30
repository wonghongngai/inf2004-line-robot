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
/* Integrated driving waits for healthy sensors and stops on faults. */
#define ALL_TEST_CONTINUE_ON_SENSOR 0
#endif
#define ALL_TEST_PWM_RAMP_PER_SECOND 100.0f
#ifndef ALL_TEST_WHITE_FOLLOW
/* Centre/right bound the white track; left input is diagnostic only. */
#define ALL_TEST_WHITE_FOLLOW 1
#endif
#ifndef ALL_TEST_CORNER_SEARCH
#define ALL_TEST_CORNER_SEARCH 1
#endif
/* Zero disables the search deadline for hardware trials. */
#define ALL_TEST_CORNER_TIMEOUT_MS 0u
#define ALL_TEST_CORNER_CENTRE_MS 60u
#define ALL_TEST_DIRECTION_HOLD_MS 300u
#ifndef ALL_TEST_STOP_ON_COLLISION
#define ALL_TEST_STOP_ON_COLLISION 1
#endif
/* Combined hardware test; fixed PWM, no calibrated distance/angle motion. */
ER all_buddies_test_start(void);
#endif

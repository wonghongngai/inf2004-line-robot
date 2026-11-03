#ifndef ALL_BUDDIES_TEST_H
#define ALL_BUDDIES_TEST_H
#include "../app.h"
#include "../subsystem3/line_barcode.h"
#ifndef ALL_TEST_LINE_LAYOUT
/* SIDE: GP7 is the offset barcode scanner, GP27/GP0 steer. TRACK disables
 * barcode decoding (GP7 over the track as a third line sensor). */
#define ALL_TEST_LINE_LAYOUT LINE_BARCODE_LAYOUT_SIDE
#endif
#ifndef ALL_TEST_BARCODE_TURNS
/* A stored barcode letter turns the next C/R-both-black junction into the
 * mapped movement instead of a stop. */
#define ALL_TEST_BARCODE_TURNS 1
#endif
#ifndef ALL_TEST_DEFAULT_MAP
/* MQTT builds wait for <client-id>/mapping; offline builds use A/B/C/Z. */
#if TM_MQTT
#define ALL_TEST_DEFAULT_MAP 0
#else
#define ALL_TEST_DEFAULT_MAP 1
#endif
#endif
/* Junction movement, tune on hardware: forward until the wheels reach the
 * crossing, then a one-wheel pivot measured with S2's turn calibration. */
#define ALL_TEST_TURN_ADVANCE_CM 5.0f
#define ALL_TEST_TURN_TIMEOUT_MS 8000u
/* After a movement, C/R both black drives straight instead of stopping. */
#define ALL_TEST_TURN_SETTLE_MS 500u
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

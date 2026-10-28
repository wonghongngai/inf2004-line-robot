#ifndef ALL_BUDDIES_TEST_H
#define ALL_BUDDIES_TEST_H
#include "../app.h"
#include "../subsystem3/line_barcode.h"
#ifndef ALL_TEST_LINE_LAYOUT
/* GP7=left, GP27=centre, GP0=right, all mounted over the track. */
#define ALL_TEST_LINE_LAYOUT LINE_BARCODE_LAYOUT_TRACK
#endif
#ifndef ALL_TEST_DURATION_MS
/* Zero keeps following until pause, line loss, junction or sensor fault. */
#define ALL_TEST_DURATION_MS 0u
#endif
/* Combined hardware test; fixed PWM, no calibrated distance/angle motion. */
ER all_buddies_test_start(void);
#endif

#ifndef S2_LINE_FOLLOW_OUTPUT_H
#define S2_LINE_FOLLOW_OUTPUT_H
#include "../subsystem3/line_barcode.h"

/* Physical L/C/R inputs occupy bits 2/1/0. In TRACK layout, the centre
 * sensor alone is straight ahead. SIDE is the separate offset-scanner
 * mounting, where only C/R steer. False always returns zero motor output.
 */
static inline bool line_follow_output(uint8_t sensors, uint8_t layout,
                                     float base, float inner,
                                     float *left, float *right)
{
    *left = *right = 0.0f;
    sensors &= LINE_BARCODE_SENSOR_MASK;
    if (layout == LINE_BARCODE_LAYOUT_SIDE) {
        sensors &= 3u;
        if (sensors == 0u) return false;
        *left = sensors == 2u ? inner : base;
        *right = sensors == 1u ? inner : base;
    } else {
        if (sensors == 0u || sensors == 5u || sensors == 7u) return false;
        *left = (sensors & LINE_BARCODE_LEFT) ? inner : base;
        *right = (sensors & LINE_BARCODE_RIGHT) ? inner : base;
    }
    return true;
}
#endif

/* Portable wheel-distance conversion shared by the motor-owning tasks. */
#ifndef S2_WHEEL_ODOMETRY_H
#define S2_WHEEL_ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>
#include <float.h>

/* Avoid libc math headers: this kernel supplies its own size_t definition. */
static inline bool wheel_odometry_finite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

typedef struct {
    int32_t left_previous, right_previous;
    float left_ticks_per_cm, right_ticks_per_cm, remainder_mm;
    bool available;
} WHEEL_ODOMETRY;

static inline void wheel_odometry_init(WHEEL_ODOMETRY *o, float left_scale,
                                      float right_scale, int32_t left, int32_t right)
{
    *o = (WHEEL_ODOMETRY){.left_previous = left, .right_previous = right,
        .left_ticks_per_cm = left_scale, .right_ticks_per_cm = right_scale,
        .available = wheel_odometry_finite(left_scale) && wheel_odometry_finite(right_scale) &&
                     left_scale > 0.0f && right_scale > 0.0f};
}

/* Rebase when stationary/turning; never feed hand movement or pivots to S4.
 * Encoder mounting may reverse either count direction, so use magnitudes.
 * Keep fractional millimetres until enough travel exists for S4's byte API.
 * Reject >255 mm per control update rather than wrapping the distance byte.
 */
static inline bool wheel_odometry_step(WHEEL_ODOMETRY *o, int32_t left,
                                      int32_t right, bool travelling, uint8_t *mm)
{
    int64_t dl = (int64_t)left - o->left_previous;
    int64_t dr = (int64_t)right - o->right_previous;
    o->left_previous = left;
    o->right_previous = right;
    *mm = 0u;
    if (!travelling || !o->available) {
        o->remainder_mm = 0.0f;
        return true;
    }
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    float distance = o->remainder_mm +
        5.0f * ((float)dl / o->left_ticks_per_cm + (float)dr / o->right_ticks_per_cm);
    if (!wheel_odometry_finite(distance) || distance >= 256.0f) return false;
    *mm = (uint8_t)distance;
    o->remainder_mm = distance - *mm;
    return true;
}
#endif

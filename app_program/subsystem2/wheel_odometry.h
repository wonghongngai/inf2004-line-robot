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

typedef struct
{
    int32_t left_previous, right_previous;
    float   left_ticks_per_cm, right_ticks_per_cm, remainder_mm;
    bool    available;
} WHEEL_ODOMETRY;

static inline void wheel_odometry_init(WHEEL_ODOMETRY * p_odo, float left_scale,
                                       float right_scale, int32_t left,
                                       int32_t right)
{
    *p_odo =
        (WHEEL_ODOMETRY){.left_previous      = left,
                         .right_previous     = right,
                         .left_ticks_per_cm  = left_scale,
                         .right_ticks_per_cm = right_scale,
                         .available = wheel_odometry_finite(left_scale) &&
                                      wheel_odometry_finite(right_scale) &&
                                      left_scale > 0.0f && right_scale > 0.0f};
}

/* Rebase when stationary/turning; never feed hand movement or pivots to S4.
 * Encoder mounting may reverse either count direction, so use magnitudes.
 * Keep fractional millimetres until enough travel exists for S4's byte API.
 * Reject >255 mm per control update rather than wrapping the distance byte.
 */
static inline bool wheel_odometry_step(WHEEL_ODOMETRY * p_odo, int32_t left,
                                       int32_t right, bool b_travelling,
                                       uint8_t * p_step_mm)
{
    int64_t delta_left    = (int64_t)left - p_odo->left_previous;
    int64_t delta_right   = (int64_t)right - p_odo->right_previous;
    p_odo->left_previous  = left;
    p_odo->right_previous = right;
    *p_step_mm            = 0u;
    if (!b_travelling || !p_odo->available)
    {
        p_odo->remainder_mm = 0.0f;
        return true;
    }
    if (delta_left < 0)
    {
        delta_left = -delta_left;
    }
    if (delta_right < 0)
    {
        delta_right = -delta_right;
    }
    float distance = p_odo->remainder_mm +
                     5.0f * ((float)delta_left / p_odo->left_ticks_per_cm +
                             (float)delta_right / p_odo->right_ticks_per_cm);
    if (!wheel_odometry_finite(distance) || distance >= 256.0f)
    {
        return false;
    }
    *p_step_mm          = (uint8_t)distance;
    p_odo->remainder_mm = distance - *p_step_mm;
    return true;
}
#endif

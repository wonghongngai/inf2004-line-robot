/**
 * @file    obstacle_logic.c
 * @brief   Subsystem 5 decision logic.  See obstacle_logic.h.
 */

#include "subsystem5.h"
#include "obstacle_logic.h"
#include "drv_servo.h"

#define HISTORY_MASK      (0x07U) /* keep 3 bits */
#define CONFIRM_THRESHOLD (2U)    /* 2 of the last 3 */

/* Median of three values: the one in the middle. */
static UW median3(UW a, UW b, UW c)
{
    UW temp;

    /* Sort a <= b <= c with three compare-and-swaps, then b is the median */
    if (a > b)
    {
        temp = a;
        a    = b;
        b    = temp;
    }
    if (b > c)
    {
        temp = b;
        b    = c;
        c    = temp;
    }
    if (a > b)
    {
        temp = a;
        a    = b;
        b    = temp;
    }

    return b;
}

EXPORT BOOL fine_scan_window(const scan_point_t * p_coarse, UW count,
                             UW * p_start_deg, UW * p_end_deg)
{
    UW   index;
    UW   min_deg     = SCAN_MAX_DEG;
    UW   max_deg     = SCAN_MIN_DEG;
    UW   closest_mm  = D_PROFILE_MM;
    UW   closest_deg = SERVO_ANGLE_CENTRE_DEG;
    BOOL b_found     = FALSE;

    if ((NULL == p_coarse) || (NULL == p_start_deg) || (NULL == p_end_deg))
    {
        return FALSE;
    }

    /* 1. Find the smallest and largest angle that saw something near,
     *    and the closest hit (used if the window must be cut to budget) */
    for (index = 0U; index < count; index++)
    {
        if ((ULTRASONIC_OK == p_coarse[index].status) &&
            (p_coarse[index].distance_mm < D_PROFILE_MM))
        {
            b_found = TRUE;
            if (p_coarse[index].angle_deg < min_deg)
            {
                min_deg = p_coarse[index].angle_deg;
            }
            if (p_coarse[index].angle_deg > max_deg)
            {
                max_deg = p_coarse[index].angle_deg;
            }
            if (p_coarse[index].distance_mm < closest_mm)
            {
                closest_mm  = p_coarse[index].distance_mm;
                closest_deg = p_coarse[index].angle_deg;
            }
        }
    }
    if (!b_found)
    {
        return FALSE;
    }

    /* 2. Widen by the margin, without going outside 30..150.
     *    unsigned values must never go below 0 */
    *p_start_deg = (min_deg >= (SCAN_MIN_DEG + FINE_MARGIN_DEG))
                       ? (min_deg - FINE_MARGIN_DEG)
                       : SCAN_MIN_DEG;
    *p_end_deg   = ((max_deg + FINE_MARGIN_DEG) <= SCAN_MAX_DEG)
                       ? (max_deg + FINE_MARGIN_DEG)
                       : SCAN_MAX_DEG;

    /* 3. Too many points for the time budget (NFR5-3)?  Centre the window
     *    on the closest hit instead.  End >= start here, so no underflow. */
    if ((((*p_end_deg - *p_start_deg) / FINE_STEP_DEG) + 1U) >
        FINE_BUDGET_POINTS)
    {
        *p_start_deg = (closest_deg >= (SCAN_MIN_DEG + FINE_HALF_SPAN_DEG))
                           ? (closest_deg - FINE_HALF_SPAN_DEG)
                           : SCAN_MIN_DEG;
        *p_end_deg   = ((closest_deg + FINE_HALF_SPAN_DEG) <= SCAN_MAX_DEG)
                           ? (closest_deg + FINE_HALF_SPAN_DEG)
                           : SCAN_MAX_DEG;
    }

    return TRUE;
}

EXPORT void scan_median3(const scan_point_t * p_in, scan_point_t * p_out,
                         UW count)
{
    UW index;

    if ((NULL == p_in) || (NULL == p_out))
    {
        return;
    }

    /* Start with an exact copy: end points and invalid points stay as-is */
    for (index = 0U; index < count; index++)
    {
        p_out[index] = p_in[index];
    }

    /* Smooth inside points that have valid neighbours */
    for (index = 1U; (index + 1U) < count; index++)
    {
        if ((ULTRASONIC_OK == p_in[index - 1U].status) &&
            (ULTRASONIC_OK == p_in[index].status) &&
            (ULTRASONIC_OK == p_in[index + 1U].status))
        {
            p_out[index].distance_mm =
                median3(p_in[index - 1U].distance_mm, p_in[index].distance_mm,
                        p_in[index + 1U].distance_mm);
        }
    }
}

EXPORT BOOL obstacle_filter_update(UB * p_history, ultrasonic_status_t status,
                                   UW distance_mm)
{
    UB b_close;
    UB count;

    if (NULL == p_history)
    {
        return FALSE;
    }

    if ((ULTRASONIC_TOO_NEAR == status) || (ULTRASONIC_BUSY == status))
    {
        /* Unknown readings, do not change history */
        b_close = 0xFFU; /* marker to skip this */
    }
    else if ((ULTRASONIC_OK == status) && (distance_mm < D_DETECT_MM))
    {
        b_close = 1U;
    }
    else
    {
        b_close = 0U;
    }

    /* shift answer into 3-bit memory */
    if (0xFFU != b_close)
    {
        *p_history = (UB)(((*p_history << 1U) | b_close) & HISTORY_MASK);
    }

    /* count the bits that indicate close to obstacle */
    count = (UB)((*p_history & 1U) + ((*p_history >> 1U) & 1U) +
                 ((*p_history >> 2U) & 1U));

    return (count >= CONFIRM_THRESHOLD) ? TRUE : FALSE;
}
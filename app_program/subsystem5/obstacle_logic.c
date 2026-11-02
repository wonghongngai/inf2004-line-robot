/**
 * @file    obstacle_logic.c
 * @brief   Subsystem 5 decision logic.  See obstacle_logic.h.
 */

#include "subsystem5.h"
#include "obstacle_logic.h"
#include "drv_servo.h"

#define HISTORY_MASK      (0x07U) /* keep 3 bits */
#define CONFIRM_THRESHOLD (2U)    /* 2 of the last 3 */

/* sin and cos x 1000 for phi = 0, 5, ... 60 deg (scans stay within +/-60) */
#define TRIG_SCALE        (1000)
#define TRIG_STEP_DEG     (5)
#define TRIG_MAX_DEG      (60)
#define BEAM_SPREAD_X1000 (263) /* 2 x tan(7.5 deg) = 0.263: cone width */

static const INT g_sin_x1000[] = {0,   87,  174, 259, 342, 423, 500,
                                  574, 643, 707, 766, 819, 866};
static const INT g_cos_x1000[] = {1000, 996, 985, 966, 940, 906, 866,
                                  819,  766, 707, 643, 574, 500};

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

static BOOL is_hit(const scan_point_t * p_point)
{
    return ((ULTRASONIC_OK == p_point->status) &&
            (p_point->distance_mm < D_PROFILE_MM))
               ? TRUE
               : FALSE;
}

static void point_to_xy(UW angle_deg, UW distance_mm, INT * p_x_mm,
                        INT * p_y_mm)
{
    INT phi_deg = (INT)angle_deg - (INT)SERVO_ANGLE_CENTRE_DEG;
    INT abs_phi = (phi_deg < 0) ? -phi_deg : phi_deg;
    INT row;

    if (abs_phi > TRIG_MAX_DEG)
    {
        abs_phi = TRIG_MAX_DEG;
    }
    row = abs_phi / TRIG_STEP_DEG;

    *p_x_mm = ((INT)distance_mm * g_cos_x1000[row]) / TRIG_SCALE;
    *p_y_mm = ((INT)distance_mm * g_sin_x1000[row]) / TRIG_SCALE;
    if (phi_deg < 0)
    {
        *p_y_mm = -*p_y_mm;
    }
}

static void clearance_update(const scan_point_t * p_point, INT x_limit_mm,
                             INT y_left_mm, INT y_right_mm,
                             obstacle_profile_t * p_out)
{
    INT x_mm;
    INT y_mm;
    UW  gap_mm;

    if (ULTRASONIC_OK != p_point->status)
    {
        return; /* only valid readings can limit clearance (NFR5-5) */
    }

    point_to_xy(p_point->angle_deg, p_point->distance_mm, &x_mm, &y_mm);
    if (x_mm > x_limit_mm)
    {
        return; /* too far ahead to block a sidestep */
    }

    if (y_mm > y_left_mm)
    {
        gap_mm = (UW)(y_mm - y_left_mm);
        if (gap_mm < p_out->clear_left_mm)
        {
            p_out->clear_left_mm = gap_mm;
        }
    }
    else if (y_mm < y_right_mm)
    {
        gap_mm = (UW)(y_right_mm - y_mm);
        if (gap_mm < p_out->clear_right_mm)
        {
            p_out->clear_right_mm = gap_mm;
        }
    }
    else
    {
        /* Level with the obstacle itself (e.g. behind it): not a gap */
    }
}

EXPORT BOOL obstacle_profile(const scan_point_t * p_fine, UW fine_count,
                             const scan_point_t * p_coarse, UW coarse_count,
                             obstacle_profile_t * p_out)
{
    UW  index;
    UW  closest_index = 0U;
    UW  first;
    UW  last;
    INT x_mm;
    INT y_mm;
    INT y_left_mm;
    INT y_right_mm;
    INT width_mm;
    INT x_limit_mm;

    if ((NULL == p_fine) || (NULL == p_coarse) || (NULL == p_out))
    {
        return FALSE;
    }

    p_out->b_valid        = FALSE;
    p_out->closest_mm     = D_PROFILE_MM;
    p_out->clear_left_mm  = CLEARANCE_OPEN_MM;
    p_out->clear_right_mm = CLEARANCE_OPEN_MM;

    /* 1. Closest hit in the fine scan */
    for (index = 0U; index < fine_count; index++)
    {
        if (is_hit(&p_fine[index]) &&
            (p_fine[index].distance_mm < p_out->closest_mm))
        {
            p_out->closest_mm = p_fine[index].distance_mm;
            closest_index     = index;
            p_out->b_valid    = TRUE;
        }
    }
    if (!p_out->b_valid)
    {
        return FALSE;
    }

    /* 2. Grow the run of neighbouring hits around it: that is the obstacle */
    first = closest_index;
    while ((first > 0U) && is_hit(&p_fine[first - 1U]))
    {
        first--;
    }
    last = closest_index;
    while (((last + 1U) < fine_count) && is_hit(&p_fine[last + 1U]))
    {
        last++;
    }

    /* 3. Left and right edges (largest and smallest y in the run) */
    y_left_mm  = -(INT)CLEARANCE_OPEN_MM;
    y_right_mm = (INT)CLEARANCE_OPEN_MM;
    for (index = first; index <= last; index++)
    {
        point_to_xy(p_fine[index].angle_deg, p_fine[index].distance_mm, &x_mm,
                    &y_mm);
        if (y_mm > y_left_mm)
        {
            y_left_mm = y_mm;
        }
        if (y_mm < y_right_mm)
        {
            y_right_mm = y_mm;
        }
    }

    /* 4. Closest point, location and beam-corrected width */
    p_out->closest_deg = p_fine[closest_index].angle_deg;
    point_to_xy(p_out->closest_deg, p_out->closest_mm, &x_mm, &y_mm);
    p_out->closest_x_mm = x_mm;
    p_out->centre_y_mm  = (y_left_mm + y_right_mm) / 2;

    width_mm = (y_left_mm - y_right_mm) -
               (((INT)p_out->closest_mm * BEAM_SPREAD_X1000) / TRIG_SCALE);
    p_out->width_mm = (width_mm > 0) ? (UW)width_mm : 0U;

    /* 5. Clearance: other objects level with this one, on either side.
     *    Fine points outside the run, then coarse points outside the
     *    fine window (the fine scan may not reach that far sideways). */
    x_limit_mm = p_out->closest_x_mm + (INT)CLEARANCE_DEPTH_MM;

    for (index = 0U; index < fine_count; index++)
    {
        if ((index < first) || (index > last))
        {
            clearance_update(&p_fine[index], x_limit_mm, y_left_mm, y_right_mm,
                             p_out);
        }
    }
    for (index = 0U; index < coarse_count; index++)
    {
        if ((0U == fine_count) ||
            (p_coarse[index].angle_deg < p_fine[0].angle_deg) ||
            (p_coarse[index].angle_deg > p_fine[fine_count - 1U].angle_deg))
        {
            clearance_update(&p_coarse[index], x_limit_mm, y_left_mm,
                             y_right_mm, p_out);
        }
    }

    return TRUE;
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

EXPORT void obstacle_plan(const obstacle_profile_t * p_profile, UW retries,
                          plan_t * p_plan)
{
    INT  half_width_mm;
    INT  y_left_mm;
    INT  y_right_mm;
    UW   shift_left_mm;
    UW   shift_right_mm;
    BOOL b_left_fits;
    BOOL b_right_fits;

    if (NULL == p_plan)
    {
        return;
    }

    /* Safe default: if anything below can't decide, stop */
    p_plan->decision = PLAN_STOP;
    p_plan->shift_mm = 0U;

    if (NULL == p_profile)
    {
        return;
    }
    if (!p_profile->b_valid)
    {
        p_plan->decision = PLAN_CONTINUE; /* nothing was found */
        return;
    }

    /* Edges from the beam-corrected width */
    half_width_mm = (INT)(p_profile->width_mm / 2U);
    y_left_mm     = p_profile->centre_y_mm + half_width_mm;
    y_right_mm    = p_profile->centre_y_mm - half_width_mm;

    /* Rule 1: entirely outside the robot's corridor? (A5-1) */
    if ((y_right_mm > (INT)CORRIDOR_HALF_MM) ||
        (y_left_mm < -(INT)CORRIDOR_HALF_MM))
    {
        p_plan->decision = PLAN_CONTINUE;
        return;
    }

    /* Rule 2: shift needed for each side.  Inside the corridor,
     * y_left >= -CORRIDOR_HALF and y_right <= +CORRIDOR_HALF,
     * so neither result can be negative. */
    shift_left_mm  = (UW)(y_left_mm + (INT)CORRIDOR_HALF_MM);
    shift_right_mm = (UW)((INT)CORRIDOR_HALF_MM - y_right_mm);

    /* Rule 3: is there room for the robot on each side? */
    b_left_fits  = (p_profile->clear_left_mm >= PASS_GAP_MM) ? TRUE : FALSE;
    b_right_fits = (p_profile->clear_right_mm >= PASS_GAP_MM) ? TRUE : FALSE;

    /* Rule 4: choose */
    if (b_left_fits && b_right_fits)
    {
        if ((shift_left_mm < shift_right_mm) ||
            ((shift_left_mm == shift_right_mm) &&
             (p_profile->clear_left_mm >= p_profile->clear_right_mm)))
        {
            p_plan->decision = PLAN_PASS_LEFT;
            p_plan->shift_mm = shift_left_mm;
        }
        else
        {
            p_plan->decision = PLAN_PASS_RIGHT;
            p_plan->shift_mm = shift_right_mm;
        }
    }
    else if (b_left_fits)
    {
        p_plan->decision = PLAN_PASS_LEFT;
        p_plan->shift_mm = shift_left_mm;
    }
    else if (b_right_fits)
    {
        p_plan->decision = PLAN_PASS_RIGHT;
        p_plan->shift_mm = shift_right_mm;
    }
    else
    {
        p_plan->decision = (retries < MAX_RETRIES) ? PLAN_REVERSE : PLAN_STOP;
    }
}
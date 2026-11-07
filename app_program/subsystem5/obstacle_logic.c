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
/* Echo cone half-width per mm of range: tan(20 deg) = 0.364.  Each edge the
 * scan sees is pulled in by this x the closest distance.  The datasheet's
 * 15 deg cone was far too narrow: 2026-11-06 a 210 mm box at 294 mm spanned
 * 415 mm of readings (415 - 2 x 0.364 x 294 = 201 mm). */
#define BEAM_HALF_X1000   (364)

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

/* Pick the angle range for the fine scan from the coarse scan */
EXPORT BOOL fine_scan_window(const scan_point_t * p_coarse, UW count,
                             UW * p_start_deg, UW * p_end_deg)
{
    UW   index;
    UW   min_deg = SCAN_MAX_DEG;
    UW   max_deg = SCAN_MIN_DEG;
    BOOL b_found = FALSE;

    if ((NULL == p_coarse) || (NULL == p_start_deg) || (NULL == p_end_deg))
    {
        return FALSE;
    }

    /* 1. Smallest and largest angle that saw something near */
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
        }
    }
    if (!b_found)
    {
        return FALSE;
    }

    /* 2. The obstacle's edges lie somewhere between the outermost hits and
     *    the next coarse angles out, so widen by one coarse step each side
     *    (2026-11-05: stopping at the last hit left the box's real left
     *    edge unscanned and the robot clipped it).  Stay inside
     *    SCAN_MIN..SCAN_MAX; unsigned values must never go below 0. */
    *p_start_deg = (min_deg >= (SCAN_MIN_DEG + COARSE_STEP_DEG))
                       ? (min_deg - COARSE_STEP_DEG)
                       : SCAN_MIN_DEG;
    *p_end_deg   = ((max_deg + COARSE_STEP_DEG) <= SCAN_MAX_DEG)
                       ? (max_deg + COARSE_STEP_DEG)
                       : SCAN_MAX_DEG;
    return TRUE;
}

/* Smooth scan readings with a 3-point median filter */
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

/* Check if a scan point is a valid close reading */
static BOOL is_hit(const scan_point_t * p_point)
{
    return ((ULTRASONIC_OK == p_point->status) &&
            (p_point->distance_mm < D_PROFILE_MM))
               ? TRUE
               : FALSE;
}

/* Check if a scan point belongs to the box whose closest reading is
 * 'closest_mm': a hit no more than NEAR_BAND_MM further than that */
static BOOL is_near(const scan_point_t * p_point, UW closest_mm)
{
    return (is_hit(p_point) &&
            (p_point->distance_mm <= (closest_mm + NEAR_BAND_MM)))
               ? TRUE
               : FALSE;
}

/* TRUE if two readings agree: both valid and within CLEAR_CONFIRM_MM */
static BOOL readings_agree(const scan_point_t * p_a, const scan_point_t * p_b)
{
    UW diff_mm;

    if ((ULTRASONIC_OK != p_a->status) || (ULTRASONIC_OK != p_b->status))
    {
        return FALSE;
    }
    diff_mm = (p_a->distance_mm > p_b->distance_mm)
                  ? (p_a->distance_mm - p_b->distance_mm)
                  : (p_b->distance_mm - p_a->distance_mm);
    return (diff_mm <= CLEAR_CONFIRM_MM) ? TRUE : FALSE;
}

/* TRUE if fine point 'index' is backed up by a neighbouring angle */
static BOOL confirmed(const scan_point_t * p_points, UW count, UW index)
{
    if ((index > 0U) && readings_agree(&p_points[index], &p_points[index - 1U]))
    {
        return TRUE;
    }
    if (((index + 1U) < count) &&
        readings_agree(&p_points[index], &p_points[index + 1U]))
    {
        return TRUE;
    }
    return FALSE;
}

/* Convert an angle and distance to x/y coordinates */
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

/* Update the left/right clearance from one scan point */
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

/* Build the obstacle's profile from the scan points */
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
    INT beam_half_mm;
    INT y_left_box_mm;
    INT y_right_box_mm;

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

    /* 2. Grow the run of neighbouring readings near the closest one: that
     *    is the obstacle.  A farther surface seen past its edge is not. */
    first = closest_index;
    while ((first > 0U) && is_near(&p_fine[first - 1U], p_out->closest_mm))
    {
        first--;
    }
    last = closest_index;
    while (((last + 1U) < fine_count) &&
           is_near(&p_fine[last + 1U], p_out->closest_mm))
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

    /* 4. Closest point, location and beam-corrected width.  An edge is
     *    pulled in by the cone's half-width only if the scan saw past it
     *    (a reading beyond the run).  A run that reaches the end of the fine
     *    scan may go on further: keep that edge where the last reading was
     *    (2026-11-06: correcting a box cut off at 50 deg gave 75 mm for a
     *    210 mm box). */
    p_out->closest_deg = p_fine[closest_index].angle_deg;
    point_to_xy(p_out->closest_deg, p_out->closest_mm, &x_mm, &y_mm);
    p_out->closest_x_mm = x_mm;

    beam_half_mm = ((INT)p_out->closest_mm * BEAM_HALF_X1000) / TRIG_SCALE;
    y_left_box_mm  = y_left_mm;
    y_right_box_mm = y_right_mm;
    if ((last + 1U) < fine_count)
    {
        y_left_box_mm -= beam_half_mm; /* left edge seen (higher angles) */
    }
    if (first > 0U)
    {
        y_right_box_mm += beam_half_mm; /* right edge seen */
    }
    if (y_left_box_mm < y_right_box_mm)
    {
        /* Narrower than the cone: a post.  Collapse to its middle. */
        y_left_box_mm  = (y_left_box_mm + y_right_box_mm) / 2;
        y_right_box_mm = y_left_box_mm;
    }
    p_out->centre_y_mm = (y_left_box_mm + y_right_box_mm) / 2;
    width_mm           = y_left_box_mm - y_right_box_mm;
    p_out->width_mm    = (UW)width_mm;

    /* 5. Clearance: other objects level with this one, on either side.
     *    Fine points outside the run, then coarse points outside the
     *    fine window (the fine scan may not reach that far sideways). */
    x_limit_mm = p_out->closest_x_mm + (INT)CLEARANCE_DEPTH_MM;

    for (index = 0U; index < fine_count; index++)
    {
        /* Only readings a neighbouring angle confirms (NFR5-5) */
        if (((index < first) || (index > last)) &&
            confirmed(p_fine, fine_count, index))
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

/* Confirm an obstacle from the last three readings */
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

/* Decide how to get around the obstacle */
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
/* ---- Bypass (FR5-4, FR5-5) ---- */

/* Move list for a left pass; a right pass swaps the turn directions */
static const bypass_step_t g_left_pass[BYPASS_STEPS] = {
    {S5_MOVE_TURN_L, BYPASS_TURN_DEG, FALSE, FALSE},  /* T1 away from line */
    {S5_MOVE_FWD, 0U, FALSE, FALSE},                  /* F1 out: from plan  */
    {S5_MOVE_TURN_R, BYPASS_TURN_DEG, FALSE, FALSE},  /* T2 forward again  */
    {S5_MOVE_FWD, BYPASS_PAST_CM, FALSE, FALSE},      /* F2 past the box   */
    {S5_MOVE_TURN_R, BYPASS_TURN_DEG, FALSE, FALSE},  /* T3 towards line   */
    {S5_MOVE_FWD, LINE_SEEK_MAX_CM, TRUE, TRUE},      /* F3 both sensors   */
                                                      /*    over the line  */
    {S5_MOVE_FWD, LINE_ALIGN_CM, FALSE, FALSE},       /* F4 wheels past it */
    {S5_MOVE_TURN_L, LINE_TURN_MAX_DEG, TRUE, FALSE}, /* T4 until a sensor */
                                                      /*    is on it again */
};

#define BYPASS_F1_INDEX (1U)
#define MM_PER_CM       (10U)

/* Step F1's distance, from the plan's shift */
EXPORT UW bypass_side_cm(UW shift_mm)
{
    UW side_cm = (shift_mm + (MM_PER_CM - 1U)) / MM_PER_CM; /* round up */

    if (side_cm < BYPASS_SIDE_MIN_CM)
    {
        side_cm = BYPASS_SIDE_MIN_CM;
    }
    if (side_cm > BYPASS_SIDE_MAX_CM)
    {
        side_cm = BYPASS_SIDE_MAX_CM;
    }
    return side_cm;
}

/* One move of the turn-and-box path */
EXPORT BOOL bypass_step(const plan_t * p_plan, UW index,
                        bypass_step_t * p_step)
{
    bypass_step_t step;

    if ((NULL == p_plan) || (NULL == p_step) || (index >= BYPASS_STEPS) ||
        ((PLAN_PASS_LEFT != p_plan->decision) &&
         (PLAN_PASS_RIGHT != p_plan->decision)))
    {
        return FALSE;
    }

    step = g_left_pass[index];
    if (BYPASS_F1_INDEX == index)
    {
        step.value = (UH)bypass_side_cm(p_plan->shift_mm);
    }
    if (PLAN_PASS_RIGHT == p_plan->decision)
    {
        if (S5_MOVE_TURN_L == step.move)
        {
            step.move = S5_MOVE_TURN_R;
        }
        else if (S5_MOVE_TURN_R == step.move)
        {
            step.move = S5_MOVE_TURN_L;
        }
        else
        {
            /* forward legs are the same both ways */
        }
    }
    *p_step = step;
    return TRUE;
}

/* ---- S5 -> VC request word (IF-10) ---- */

/* Request word layout (see subsystem5.h) */
#define REQ_KIND_SHIFT  (0U)
#define REQ_KIND_MASK   (0x07U)
#define REQ_MOVE_SHIFT  (3U)
#define REQ_MOVE_MASK   (0x03U)
#define REQ_LINE_SHIFT  (5U)
#define REQ_SEQ_SHIFT   (6U)
#define REQ_VALUE_SHIFT (10U)
#define REQ_VALUE_MASK  (0x3FFU)
#define REQ_UNUSED_LOW_SHIFT  (20U) /* bits 20-27 must be 0 */
#define REQ_UNUSED_LOW_MASK   (0xFFU)
#define REQ_MOVE_RESERVED     (3U)  /* move 3: invalid */
#define REQ_LALL_SHIFT  (28U) /* line_all */
#define REQ_UNUSED_SHIFT (29U)

/* Pack a request into one 32-bit word */
EXPORT UW s5_request_pack(const S5_REQUEST * p_req)
{
    UW value;

    if (NULL == p_req)
    {
        return (UW)S5_REQ_SAFE_STOP; /* never publish garbage */
    }

    value = (p_req->value > S5_REQ_VALUE_MAX) ? S5_REQ_VALUE_MAX
                                              : (UW)p_req->value;

    return (((UW)p_req->kind & REQ_KIND_MASK) << REQ_KIND_SHIFT) |
           (((UW)p_req->move & REQ_MOVE_MASK) << REQ_MOVE_SHIFT) |
           ((p_req->b_stop_on_line ? 1UL : 0UL) << REQ_LINE_SHIFT) |
           (((UW)p_req->seq & S5_REQ_SEQ_MASK) << REQ_SEQ_SHIFT) |
           ((value & REQ_VALUE_MASK) << REQ_VALUE_SHIFT) |
           ((p_req->b_line_all ? 1UL : 0UL) << REQ_LALL_SHIFT);
}

/* Unpack a request word, rejecting reserved values */
EXPORT BOOL s5_request_unpack(UW word, S5_REQUEST * p_req)
{
    UB kind;
    UB move;

    if (NULL == p_req)
    {
        return FALSE;
    }

    p_req->kind           = S5_REQ_NONE;
    p_req->move           = S5_MOVE_FWD;
    p_req->seq            = 0U;
    p_req->b_stop_on_line = FALSE;
    p_req->value          = 0U;
    p_req->b_line_all     = FALSE;

    kind = (UB)((word >> REQ_KIND_SHIFT) & REQ_KIND_MASK);
    move = (UB)((word >> REQ_MOVE_SHIFT) & REQ_MOVE_MASK);

    if ((kind > S5_REQ_SAFE_STOP) || (REQ_MOVE_RESERVED == move) ||
        (0U != ((word >> REQ_UNUSED_LOW_SHIFT) & REQ_UNUSED_LOW_MASK)) ||
        (0U != (word >> REQ_UNUSED_SHIFT)))
    {
        return FALSE; /* reserved: the VC must stop safely */
    }

    p_req->kind = kind;
    p_req->move = move;
    p_req->seq  = (UB)((word >> REQ_SEQ_SHIFT) & S5_REQ_SEQ_MASK);
    p_req->b_stop_on_line =
        (0U != ((word >> REQ_LINE_SHIFT) & 1U)) ? TRUE : FALSE;
    p_req->value = (UH)((word >> REQ_VALUE_SHIFT) & REQ_VALUE_MASK);
    p_req->b_line_all =
        (0U != ((word >> REQ_LALL_SHIFT) & 1U)) ? TRUE : FALSE;
    return TRUE;
}

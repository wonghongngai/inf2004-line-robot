/**
 * @file    obstacle_logic.h
 * @brief   Subsystem 5 decision logic.  Pure functions only (no hardware,
 *          no kernel calls) so they can be unit-tested with fixed inputs.
 */

#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include <tk/tkernel.h>     /* UW, UB, BOOL */
#include "drv_ultrasonic.h" /* ultrasonic_status_t */

/* Scan geometry */
/* Scans never go outside 50..130 deg (was 30..150, then 60..120, on
 * 2026-11-05): 40 deg either side of straight ahead. */
#define SCAN_MIN_DEG    (50U)
#define SCAN_MAX_DEG    (130U)
/* Scans:
 *  - coarse (FR5-1): jump to 50, 70, 90, 110, 130 in turn, ping at each;
 *  - fine (FR5-2): glide smoothly between the coarse angles that saw the
 *    obstacle, pinging every 5 deg (e.g. hits at 70 and 90 -> 70..90).
 * The NFR5-2/NFR5-3 time limits are dropped for smoothness (2026-11-05). */
#define COARSE_STEP_DEG (20U)
#define COARSE_POINTS   (((SCAN_MAX_DEG - SCAN_MIN_DEG) / COARSE_STEP_DEG) + 1U)
#define FINE_STEP_DEG   (5U)
#define FINE_MAX_POINTS (((SCAN_MAX_DEG - SCAN_MIN_DEG) / FINE_STEP_DEG) + 1U)

#define D_DETECT_MM  (300U) /* detect at >= 30 cm */
/* A reading counts as "something there" for profiling below this.
 * Further than this is treated as clear.  600 mm, not 300: the sides of
 * an obstacle 30 cm ahead are further away. */
#define D_PROFILE_MM (600U)

/* only objects up to this far beyond the closest point limit a sidestep */
#define CLEARANCE_DEPTH_MM (500U)
#define CLEARANCE_OPEN_MM  (9999U) /* "nothing seen on that side" */
#define ROBOT_WIDTH_MM (150U) /* W_R: robot's widest point [VERIFY: measure] */
#define PASS_MARGIN_MM (50U)  /* M: space kept from obstacles [VERIFY]       */
#define CORRIDOR_HALF_MM ((ROBOT_WIDTH_MM / 2U) + PASS_MARGIN_MM) /* A5-1   */
#define PASS_GAP_MM      (ROBOT_WIDTH_MM + (2U * PASS_MARGIN_MM)) /* to fit */
#define MAX_RETRIES      (2U) /* reverse-and-rescan attempts before NO_PATH */

typedef struct
{
    UW                  angle_deg;
    UW                  distance_mm;
    ultrasonic_status_t status;
} scan_point_t;

/**
 * Obstacle description from one scan.  Positions are from the sensor:
 * x forward, y LEFT (negative = right), all in mm.
 */
typedef struct
{
    BOOL b_valid;        /* FALSE: no obstacle found, ignore the rest */
    UW   closest_mm;     /* closest point: distance ...               */
    UW   closest_deg;    /* ... and robot angle                       */
    INT  closest_x_mm;   /* how far ahead the closest point is        */
    INT  centre_y_mm;    /* sideways centre: + left, - right          */
    UW   width_mm;       /* beam-corrected width                      */
    UW   clear_left_mm;  /* gap beside the left edge, or OPEN         */
    UW   clear_right_mm; /* gap beside the right edge, or OPEN        */
} obstacle_profile_t;

typedef enum
{
    PLAN_CONTINUE = 0, /* not in the corridor: carry on               */
    PLAN_PASS_LEFT,    /* go round on the left by shift_mm            */
    PLAN_PASS_RIGHT,   /* go round on the right by shift_mm           */
    PLAN_REVERSE,      /* no room: back up and rescan                 */
    PLAN_STOP          /* no room after MAX_RETRIES (NO_PATH), or bad
                          input: stop safely                          */
} plan_decision_t;

typedef struct
{
    plan_decision_t decision;
    UW              shift_mm; /* sideways shift for PASS_LEFT/RIGHT, else 0 */
} plan_t;

/**
 * Choose the fine-scan angles from the coarse scan: from the first coarse
 * angle that saw the obstacle to the last one (hits at 60 and 90 ->
 * 70..90).  If only ONE coarse angle saw it, its edges lie somewhere
 * towards the neighbouring coarse angles, so the window runs from the
 * neighbour below to the neighbour above (hit at 90 only -> 70..110),
 * limited to SCAN_MIN_DEG..SCAN_MAX_DEG.  A hit is an ULTRASONIC_OK
 * reading under D_PROFILE_MM.
 * @return FALSE if there are no hits (obstacle gone: skip the fine scan)
 *         or a pointer is NULL.
 */
IMPORT BOOL fine_scan_window(const scan_point_t * p_coarse, UW count,
                             UW * p_start_deg, UW * p_end_deg);

/**
 * Smooth a scan by replacing each distance with the median of itself and
 * its two neighbours, but only where all three are ULTRASONIC_OK.
 * Invalid points and the two end points are copied unchanged, so no
 * reading is ever invented (NFR5-5).
 * p_in and p_out must be different arrays of 'count' points.
 */
IMPORT void scan_median3(const scan_point_t * p_in, scan_point_t * p_out,
                         UW count);

/**
 * Add one reading to the 3-reading history and report whether an
 * obstacle is confirmed (2 of the last 3 readings close).
 * Only ULTRASONIC_OK readings can count as close (NFR5-5).
 * TOO_NEAR and BUSY readings are ignored: history is left unchanged.
 * @param  p_history    In/out: last 3 answers, 1 bit each (bit 0 newest).
 * @param  status       Status of the new reading.
 * @param  distance_mm  Distance of the new reading (used only if OK).
 * @return TRUE if an obstacle is confirmed.
 */
IMPORT BOOL obstacle_filter_update(UB * p_history, ultrasonic_status_t status,
                                   UW distance_mm);

/**
 * Describe the obstacle (FR5-3) from the smoothed fine scan, using coarse
 * points outside the fine window for clearance only.
 * The obstacle is the run of neighbouring hits around the closest hit.
 * @return FALSE (and p_out->b_valid = FALSE) if the fine scan has no hit.
 */
IMPORT BOOL obstacle_profile(const scan_point_t * p_fine, UW fine_count,
                             const scan_point_t * p_coarse, UW coarse_count,
                             obstacle_profile_t * p_out);

/**
 * Choose how to avoid the profiled obstacle.
 * @param  p_profile  From obstacle_profile().
 * @param  retries    Reverse-and-rescan attempts already made.
 * @param  p_plan     Out: decision and shift.  PLAN_STOP on bad input.
 */
IMPORT void obstacle_plan(const obstacle_profile_t * p_profile, UW retries,
                          plan_t * p_plan);

#endif /* OBSTACLE_LOGIC_H */
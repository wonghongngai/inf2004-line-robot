/**
 * @file    obstacle_logic.h
 * @brief   Subsystem 5 decision logic.  Pure functions only (no hardware,
 *          no kernel calls) so they can be unit-tested with fixed inputs.
 */

#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include <tk/tkernel.h>     /* UW, UB, BOOL */
#include "drv_ultrasonic.h" /* ultrasonic_status_t */
#include "subsystem5.h"     /* S5_REQUEST, S5_REQ_*, S5_MOVE_* */

/* Scan geometry */
/* Scans never go outside 50..130 deg (was 30..150, then 60..120, on
 * 2026-11-05): 40 deg either side of straight ahead. */
#define SCAN_MIN_DEG    (50U)
#define SCAN_MAX_DEG    (130U)
/* Scans:
 *  - coarse: jump to 50, 70, 90, 110, 130 in turn, ping at each;
 *  - fine: glide smoothly from one coarse step before the first
 *    hit to one step after the last, pinging every 5 deg (e.g. hits at
 *    90 and 110 -> 70..130), so the obstacle's edges are measured. */
#define COARSE_STEP_DEG (20U)
#define COARSE_POINTS   (((SCAN_MAX_DEG - SCAN_MIN_DEG) / COARSE_STEP_DEG) + 1U)
#define FINE_STEP_DEG   (5U)
#define FINE_MAX_POINTS (((SCAN_MAX_DEG - SCAN_MIN_DEG) / FINE_STEP_DEG) + 1U)

/* Detect at 20 cm (2026-11-07; it was 30 cm from 2026-11-06, and 20 cm
 * on 2026-11-05/06).  NFR5-1 asks for >= 30 cm: this is a deliberate
 * deviation for the turn-and-box bypass, to record in the test log.
 * The robot must come to rest >= 10 cm from the obstacle, and the bypass
 * needs BYPASS_MIN_GAP_MM (half of this, 100 mm) in front of it for its
 * first pivot. */
#define D_DETECT_MM (200U)

/* A reading counts as "something there" for profiling below this.
 * Further than this is treated as clear. */
#define D_PROFILE_MM (600U)

/* Profile only (the fine-scan window still uses D_PROFILE_MM):
 * - the box is the run of readings within NEAR_BAND_MM of the closest, so
 *   a farther surface seen past its edge is not part of it;
 * - a reading outside the box only narrows the clearance beside it if a
 *   neighbouring angle confirms it (valid and within CLEAR_CONFIRM_MM),
 *   so one stray echo never decides the side (NFR5-5).
 * 2026-11-06 bench: a lone 501 mm echo at 120 deg widened the box to
 * 362 mm and the robot passed right instead of left.  [Tune] */
#define NEAR_BAND_MM     (150U)
#define CLEAR_CONFIRM_MM (100U)

/* only objects up to this far beyond the closest point limit a sidestep */
#define CLEARANCE_DEPTH_MM  (500U)
#define CLEARANCE_OPEN_MM   (9999U) /* looked past that edge: nothing there */
/* Never looked past that edge: the box's readings run on to the scan
 * limit (50 or 130 deg) on that side, so the space beyond is unknown, not
 * open (2026-11-08: a box running past 50 deg read "R 9999 = open"). */
#define CLEARANCE_UNSEEN_MM (9998U)
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
    UW   clear_left_mm;  /* gap beside the left edge, OPEN or UNSEEN  */
    UW   clear_right_mm; /* gap beside the right edge, OPEN or UNSEEN */
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
    BOOL            b_side_unseen; /* the chosen side was never seen past
                                    * the box: no side was seen clear    */
} plan_t;

/**
 * Choose the fine-scan angles from the coarse scan.  The obstacle's edges
 * lie somewhere between the outermost coarse angles that saw it and the
 * next coarse angles out, so the window runs from one coarse step below
 * the first hit to one step above the last (hits at 90 and 110 ->
 * 70..130; 90 only -> 70..110), limited to SCAN_MIN_DEG..SCAN_MAX_DEG.
 * A hit is an ULTRASONIC_OK reading under D_PROFILE_MM.
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
 * The obstacle is the run of neighbouring readings around the closest hit
 * that are within NEAR_BAND_MM of it.  Fine readings outside the run only
 * narrow the clearance if a neighbouring angle confirms them.
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

/* ---- Bypass (FR5-4, FR5-5) -------------------------------------------
 * Turn-and-box path (one-wheel 90 deg pivots and straight legs), e.g. for
 * a left pass; a right pass is the mirror image:
 *   T1 turn L 90   F1 forward the plan's shift (out to the side), see
 *                  bypass_side_cm()
 *   T2 turn R 90   F2 forward, watching the box sideways, until it has
 *                  been passed (any depth), see "Watched pass" below
 *                  [T3 is followed by a look ahead: box still in the
 *                  way -> turn back, PROBE_STEP_CM on, turn again]
 *   T3 turn R 90   F3 forward until BOTH IR sensors have crossed onto
 *                  the line, at most LINE_SEEK_MAX_CM (NFR5-4)
 *                  F4 forward LINE_ALIGN_CM more (wheels past the line)
 *   T4 turn L     until an IR sensor sees the line again, at most
 *                  LINE_TURN_MAX_DEG: then line following takes over
 *                  (RESUME).
 * F4: the IR sensors are ahead of the wheels, so when they first see the
 * line the wheels are still short of it, and pivoting on the near wheel
 * swung the sensors out beside the line: the robot ended up parallel to
 * it, 3-5 cm off (2026-11-07).  [Tune: still beside the line -> raise
 * LINE_ALIGN_CM; ends past it on the far side -> lower it]
 * F3 waits for both sensors: arriving at an angle, the first sensor to
 * touch the line stopped it with the other still short, so F4 left one
 * sensor before the line and T4 found it after only ~40 deg (2026-11-07,
 * run 9).
 * T4 ends on the sensor, not after a fixed 90: line following keeps the
 * line BETWEEN its centre and right sensors, and a fixed 90 swept both
 * sensors back across the line (R saw it just before the turn ended),
 * leaving it outside them; the follower then drove "straight", drifted
 * away and stopped on a later crossing (2026-11-07, runs 6 and 7).
 * Stopping when the first sensor reaches the line leaves the follower a
 * small correction instead.
 * F1 comes from the scan (the plan's shift); F2 from watching the box, so
 * the box may be any depth up to about PASS_MAX_CM.  [VERIFY on the
 * floor] */
#define BYPASS_TURN_DEG      (90U)
#define BYPASS_SIDE_MIN_CM   (10U)  /* F1: never less than this      */
#define BYPASS_SIDE_MAX_CM   (40U)  /* F1: never more than this      */
#define LINE_SEEK_MAX_CM     (50U)  /* NFR5-4: line within 50 cm      */
#define LINE_ALIGN_CM        (4U)   /* F4: on past the line to pivot   */
#define LINE_TURN_MAX_DEG    (120U) /* T4: turn this far at most       */
#define BYPASS_STEPS         (8U)   /* T1 F1 T2 F2 T3 F3 F4 T4         */
#define BYPASS_FRONT_STOP_MM (100U) /* F legs: closer ahead = stop     */
/* The first pivot swings the front corner forward: a box closer than
 * this (straight ahead, from the scan) leaves no room, so stop safely.
 * Half the detection distance, as before (150 of 300 mm); 100 mm now
 * the robot stops about 20 cm away (2026-11-07).  [VERIFY] */
#define BYPASS_MIN_GAP_MM    (D_DETECT_MM / 2U)

/* One move of the bypass path: an S5_MOVE_* with its value (degrees for
 * a turn, cm for a forward leg) */
typedef struct
{
    UB   move;           /* S5_MOVE_FWD, S5_MOVE_TURN_L or S5_MOVE_TURN_R */
    UH   value;          /* deg or cm                                     */
    BOOL b_stop_on_line; /* F3, T4: end the move on the line              */
    BOOL b_line_all;     /* F3: only once every sensor has seen it        */
    BOOL b_watch_side;   /* F2: watched pass, value = PASS_MAX_CM         */
    BOOL b_check_ahead;  /* T3: look ahead afterwards, retry if blocked   */
} bypass_step_t;

/**
 * Step F1's distance: the plan's sideways shift (the passing-side edge of
 * the box + half the robot + PASS_MARGIN_MM), rounded up to cm and kept
 * within BYPASS_SIDE_MIN_CM..BYPASS_SIDE_MAX_CM.  The pivots T1 and T2
 * each add about half a wheel spacing sideways as well; that is left as
 * extra margin, not counted on.
 */
IMPORT UW bypass_side_cm(UW shift_mm);

/**
 * Move 'index' (0..BYPASS_STEPS-1) of the turn-and-box path for the plan's
 * left or right pass (see the list above); F1's distance comes from the
 * plan's shift (bypass_side_cm).
 * @return FALSE (p_step unchanged) if p_plan is NULL or not a pass, the
 *         index is out of range or p_step is NULL.
 */
IMPORT BOOL bypass_step(const plan_t * p_plan, UW index,
                        bypass_step_t * p_step);

/* ---- Watched pass (F2) ------------------------------------------------
 * After T2 the robot faces forward with the box beside it (on the right
 * for a left pass).  The servo turns the sensor SIDE_LOOK_DEG towards the
 * box (50 or 130: as far as the holder turns) and F2 drives on while it
 * pings.  The echo cone is about +/-20 deg (the box-size calibration,
 * 2026-11-06), so the beam meets the box's side face from gap x cot(60)
 * to gap x cot(20) ahead of the sensor, 'gap' being how far the face is
 * out to the side.
 *   - Seen, then SIDE_CLEAR_COUNT clear readings in a row: the far end is
 *     about gap x cot(60) ahead, so drive on that + the robot's tail +
 *     PASS_MARGIN_MM (bypass_after_clear_cm), then T3.  If that happens
 *     inside the planned distance (bypass_pass_plan_cm), the planned
 *     distance is driven first anyway: F2 never ends before it.
 *   - Never seen by bypass_pass_plan_cm(): a box too shallow to reach the
 *     beam (under about gap x cot(60) - its front's distance); stop there.
 *   - Seen but not passed by PASS_MAX_CM: too long a box, stop safely.
 * The side watch can be fooled: a long smooth side meets the beam at
 * 20-60 deg and reflects the sound away, so it reads clear while the
 * robot is still beside it (2026-11-08: a 40 cm box "passed" after 1-2 cm
 * and the robot hit it).  So the look ahead after T3 decides: facing the
 * box square on, its echo is reliable.  Anything closer than
 * bypass_probe_mm(), or too close to measure, means the box is still in
 * the way: turn back, drive PROBE_STEP_CM on and turn again, at most
 * PROBE_TRIES times (PROBE_STEP_CM x PROBE_TRIES covers a PASS_MAX_CM box).
 * The pivots: each 90 deg one-wheel pivot moves the robot's centre about
 * half a wheel spacing forward and half sideways, so T1 + T2 together
 * about PIVOT_STEP_MM each way.  [VERIFY all lengths on the robot] */
#define SIDE_LOOK_DEG       (40U)  /* servo 90 -/+ this               */
#define PIVOT_STEP_MM       (100U) /* T1 + T2: forward and sideways   */
#define ROBOT_TAIL_MM       (150U) /* sensor to the robot's rear      */
#define BOX_UNSEEN_DEPTH_MM (60U)  /* depth assumed if never seen     */
#define PASS_MAX_CM         (80U)  /* longest F2 (deepest box)        */
/* T4 pivots on one wheel and swings the robot's back corner about 15 cm
 * round towards the box: drive on this much more after "passed" so the
 * corner clears the box's far end (2026-11-08: it hit the box's corner
 * in the last turn).  [VERIFY] */
#define TURN_SWEEP_MM       (100U)
/* After T3: anything straight ahead closer than gap + this is the box
 * still in the way.  2 x PASS_MARGIN_MM (100) let a 362 mm reading of the
 * box's corner through as "clear" (2026-11-08).  [VERIFY] */
#define PROBE_MARGIN_MM     (200U)
/* Clear readings in a row = past.  One reading every ~75 ms at about
 * 10 cm/s is about 0.75 cm of travel, so 10 is about 7-8 cm clear (2 was
 * about 1.5 cm and was fooled by one corner echo, 2026-11-08). */
#define SIDE_CLEAR_COUNT    (10U)
#define COT_CLEAR_X1000     (577U)  /* cot(40 + 20 deg) x 1000         */
#define INV_SIN_EDGE_X1000  (2924U) /* 1 / sin(40 - 20 deg) x 1000     */
#define SIDE_NEAR_MAX_MM    (1000U) /* "beside" never further than this */
#define PROBE_STEP_CM       (15U)   /* box still ahead: this much on   */
#define PROBE_TRIES         (5U)    /* ... at most this many times     */
                                    /* (75 cm: up to a PASS_MAX_CM box) */

/* "Passed the box" detector for the watched pass */
typedef struct
{
    BOOL b_seen;    /* the box has been seen beside the robot */
    UB   clear_run; /* clear readings in a row since then     */
} side_pass_t;

/**
 * Feed one side reading.  The box is seen when an OK reading is under
 * 'near_mm'; it has been passed after SIDE_CLEAR_COUNT clear readings in
 * a row since (OK >= near_mm, TOO_FAR or NO_ECHO).  TOO_NEAR and BUSY
 * change nothing (NFR5-5); a near reading again resets the clear count.
 * @return TRUE once the box has been passed; FALSE on NULL.
 */
IMPORT BOOL side_pass_update(side_pass_t * p_state, ultrasonic_status_t status,
                             UW distance_mm, UW near_mm);

/**
 * How far out to the side the box's face is during F2, from the sensor:
 * F1 (bypass_side_cm) + PIVOT_STEP_MM - the box's passing-side edge (centre
 * y +/- half the width).  At least 50 mm; 0 if not a pass or NULL.
 */
IMPORT UW bypass_side_gap_mm(const plan_t *             p_plan,
                             const obstacle_profile_t * p_profile);

/**
 * Side readings under this mean "the box is beside": the furthest point
 * of the face inside the beam, gap / sin(20 deg), + PASS_MARGIN_MM, at
 * most SIDE_NEAR_MAX_MM.
 */
IMPORT UW bypass_side_near_mm(UW gap_mm);

/**
 * F2's planned length if the box is never seen: the box's front (closest
 * x less PIVOT_STEP_MM) + BOX_UNSEEN_DEPTH_MM + ROBOT_TAIL_MM +
 * PASS_MARGIN_MM, in cm rounded up, within 10..PASS_MAX_CM.  10 if NULL.
 */
IMPORT UW bypass_pass_plan_cm(const obstacle_profile_t * p_profile);

/**
 * How far to drive on once the box has been passed: gap x cot(60 deg)
 * (the far end is still that far ahead) + ROBOT_TAIL_MM + PASS_MARGIN_MM
 * + TURN_SWEEP_MM (the last turn swings the back corner round), in cm
 * rounded up.
 */
IMPORT UW bypass_after_clear_cm(UW gap_mm);

/**
 * After T3: a reading straight ahead under this means the box is still in
 * the way: gap + PROBE_MARGIN_MM.
 */
IMPORT UW bypass_probe_mm(UW gap_mm);

/**
 * Servo angle that looks at the box during F2: the box is on the right
 * for a left pass (90 - 40 = 50), on the left for a right pass (130).
 * SERVO_ANGLE_CENTRE_DEG if not a pass.
 */
IMPORT UW bypass_side_deg(plan_decision_t decision);

/* ---- S5 -> VC request word (IF-10), CR1 minimum-bit packing ---- */

/**
 * Pack a request into the 32-bit word layout in subsystem5.h.
 * Out-of-range fields are cut to their bit width; value is clamped to
 * S5_REQ_VALUE_MAX.  NULL gives a SAFE_STOP word, never garbage.
 */
IMPORT UW s5_request_pack(const S5_REQUEST * p_req);

/**
 * Unpack a request word.
 * @return FALSE (and *p_req set to NONE) if a reserved value is found (kind
 *         5..7, move 3, or any of bits 20-27 or 29-31 set), or FALSE if
 *         p_req is NULL.  The VC must stop on FALSE.
 */
IMPORT BOOL s5_request_unpack(UW word, S5_REQUEST * p_req);

#endif /* OBSTACLE_LOGIC_H */
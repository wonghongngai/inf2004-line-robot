/**
 * @file    task_obstacle.c
 * @brief   obstacle_task: the S5 state machine.  Owns the HC-SR04P and the
 *          SG90, and asks the Vehicle Controller (VC) to stop or move the
 *          robot through the request word (IF-10, see subsystem5.h).  It
 *          never drives the motors itself.
 */

#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_logic.h"
#include "../subsystem4/subsystem4.h" /* IMU tilt/hump gating (A5-2) */

/* Builds with a controller that answers S5's requests (IF-10/IF-11):
 * the integrated profiles (the VC in robot_control.c) and all_test (its
 * own loop in all_buddies_test.c).  Other builds have nobody to reply, so
 * S5 must not wait for "following" or hold the robot there: it just
 * pings, scans and prints the plan (bench behaviour) and keeps watching. */
#define S5_HAS_VC (APP_ROBOT_INTEGRATED || APP_ALL_TEST)

#define MONITOR_GAP_MS    (30U)   /* + ~30 ms ping = one ping per ~70-80 ms */
#define IDLE_POLL_MS      (50U)   /* while the VC is not line-following    */
#define RESUME_QUIET_MS   (1000U) /* no detection right after resuming     */
#define STARTUP_SETTLE_MS (300U)  /* chassis still after the first swing   */
#define HOLD_TIMEOUT_MS   (1500)  /* VC must have stopped by then          */
#define MOVE_TIMEOUT_MS   (25000) /* > any controller's move time limit    */

typedef enum
{
    S5_MONITORING = 0, /* servo forward, pinging, watching for obstacles */
    S5_HOLDING,        /* obstacle confirmed: waiting for the robot to stop */
    S5_SCANNING,       /* robot still: coarse jumps + fine glide, plan    */
    S5_BYPASSING,      /* driving round the box and back to the line      */
    S5_SAFE_STOP       /* no path or a failed request: stay put           */
} s5_state_t;

/* ---- S5 -> VC request word (IF-10) ------------------------------------
 * Only this task writes it, with one aligned 32-bit store (atomic on the
 * M0+), so the VC on the other core never sees half a request. */
static volatile UW g_s5_request = 0U; /* S5_REQ_NONE */
static UB          g_seq        = 0U;

/* Read the current S5 request (called by the VC every control step) */
EXPORT ER subsystem5_get_request(S5_REQUEST * p_req)
{
    if (NULL == p_req)
    {
        return E_PAR;
    }
    return s5_request_unpack(g_s5_request, p_req) ? E_OK : E_PAR;
}

/* Publish a new request with the next sequence number */
static void s5_post_line(UB kind, UB move, UW value, BOOL b_stop_on_line,
                         BOOL b_line_all)
{
    S5_REQUEST req;

    g_seq = (UB)((g_seq + 1U) & S5_REQ_SEQ_MASK);

    req.kind           = kind;
    req.move           = move;
    req.seq            = g_seq;
    req.b_stop_on_line = b_stop_on_line;
    req.value = (UH)((value > S5_REQ_VALUE_MAX) ? S5_REQ_VALUE_MAX : value);
    req.b_line_all = (b_stop_on_line && b_line_all) ? TRUE : FALSE;

    g_s5_request = s5_request_pack(&req); /* one store */
}

/* Publish a request whose stop-on-line (if any) needs just one sensor */
static void s5_post(UB kind, UB move, UW value, BOOL b_stop_on_line)
{
    s5_post_line(kind, move, value, b_stop_on_line, FALSE);
}

/* Post a request, then wait for any of the wanted reply bits.
 * @return the reply bits that arrived, or 0 on timeout. */
static UINT s5_post_and_wait(UB kind, UB move, UW value, BOOL b_stop_on_line,
                             UINT wanted, TMO timeout_ms)
{
    UINT pattern = 0U;

    /* Forget old replies first, so a stale bit can't answer this one */
    (void)tk_clr_flg(flg_s5_evt, ~wanted);
    s5_post(kind, move, value, b_stop_on_line);

    if (E_OK != tk_wai_flg(flg_s5_evt, wanted, TWF_ORW, &pattern, timeout_ms))
    {
        return 0U;
    }
    return pattern & wanted;
}

/* Ask the VC to stop and wait until the robot is at rest */
static BOOL s5_hold(void)
{
    return (0U != s5_post_and_wait(S5_REQ_HOLD, S5_MOVE_FWD, 0U, FALSE,
                                   S5_EVT_STOPPED, HOLD_TIMEOUT_MS))
               ? TRUE
               : FALSE;
}

/* Ask the VC for one move and wait for the result.
 * @return the reply bits (MOTION_DONE, LINE_FOUND or MOVE_FAILED), 0 on
 *         timeout. */
static UINT s5_move(UB move, UW value, BOOL b_stop_on_line)
{
    UINT wanted = S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED;

    if (b_stop_on_line)
    {
        wanted |= S5_EVT_LINE_FOUND;
    }
    return s5_post_and_wait(S5_REQ_MOVE, move, value, b_stop_on_line, wanted,
                            MOVE_TIMEOUT_MS);
}

/* Check whether a forward obstacle reading may count right now */
static BOOL s5_detect_allowed(void)
{
    IMU_DATA imu;
#if S5_HAS_VC
    UINT pattern = 0U;

    /* Only while the VC is armed and following the line */
    if (E_OK !=
        tk_wai_flg(flg_s5_evt, S5_EVT_FOLLOWING, TWF_ORW, &pattern, TMO_POL))
    {
        return FALSE;
    }
#endif

    /* A5-2: a hump ramp or the floor must not look like an obstacle */
    if (E_OK != subsystem4_get_latest(&imu))
    {
        return FALSE;
    }
    if ((IMU_MOTION_HUMP == imu.motion_state) ||
        (IMU_MOTION_TILT == imu.motion_state))
    {
        return FALSE;
    }
    return TRUE;
}

/* Block until S4 reports a finished IMU calibration.  The VC won't arm
 * without one either, so waiting forever here costs nothing. */
static void wait_for_imu_calibration(void)
{
    IMU_DATA imu;

    for (;;)
    {
        if ((E_OK == subsystem4_get_latest(&imu)) && imu.calibrated)
        {
            return;
        }
        tk_dly_tsk(IDLE_POLL_MS);
    }
}

#define PING_CYCLE_US  (60000U) /* sensor: >= 60 ms between pings */
#define PING_WINDOW_US (ULTRASONIC_WINDOW_SCAN_MS * 1000U)

/* When the last ping was fired, across passes: the sensor needs
 * PING_CYCLE_US between pings even when one pass ends and the next
 * starts at the same angle. */
static UW   g_last_ping_us  = 0U;
static BOOL g_b_ever_pinged = FALSE;

/* TRUE if the stored point index is the mark the horn has now reached */
static BOOL mark_reached(UW mark_deg, BOOL b_up)
{
    UW angle_ddeg = servo_get_angle_ddeg();
    UW mark_ddeg  = mark_deg * SERVO_DDEG_PER_DEG;

    return (b_up ? (angle_ddeg >= mark_ddeg) : (angle_ddeg <= mark_ddeg))
               ? TRUE
               : FALSE;
}

/*
 * Glide smoothly from 'from_deg' to 'to_deg' (either direction), pinging
 * each time the horn reaches the next mark, every 'step_deg' from
 * 'from_deg', both ends included (the fine scan uses 5 deg).
 * Results go in ANGLE order
 * whichever way it travels: p_points[i] is min(from, to) + i * step.
 * Glides to 'from_deg' first if the servo is not already there; leaves
 * the servo at 'to_deg'.  (to - from) must be a multiple of step_deg.
 *
 * The task never sleeps for a whole echo window here: it polls every
 * SERVO_GLIDE_POLL_MS, updating the servo every time, and starts or
 * finishes a ping in between (the echo is timed in hardware).  So the
 * horn gets a fresh position every PWM period even while a ping is in
 * flight; a 20 ms blocking ping used to skip a period and make it judder.
 * At cruise, 5 deg pings land 62-74 ms apart; PING_CYCLE_US still guards
 * the sensor's 60 ms minimum.  No printing inside: it would stall it.
 * @return Number of points stored.
 */
static UW glide_and_ping(UW from_deg, UW to_deg, UW step_deg,
                         scan_point_t * p_points)
{
    const BOOL b_up    = (to_deg >= from_deg) ? TRUE : FALSE;
    const UW   low_deg = b_up ? from_deg : to_deg;
    const UW   points =
        (((b_up ? (to_deg - from_deg) : (from_deg - to_deg))) / step_deg) + 1U;
    UW   mark_deg = from_deg;
    UW   done     = 0U;
    UW   slot     = 0U;
    UW   now_us;
    BOOL b_pinging = FALSE;

    servo_glide_to(from_deg); /* no-op if already there */
    servo_glide_begin(to_deg);

    while (done < points)
    {
        (void)servo_glide_update();
        now_us = rp2040_time_us();

        if (b_pinging)
        {
            /* Echo window over: collect the reading */
            if ((now_us - g_last_ping_us) >= PING_WINDOW_US)
            {
                p_points[slot].status =
                    ultrasonic_ping_finish(&p_points[slot].distance_mm);
                b_pinging = FALSE;
                done++;
                mark_deg = b_up ? (mark_deg + step_deg) : (mark_deg - step_deg);
            }
        }
        else if (mark_reached(mark_deg, b_up) &&
                 ((!g_b_ever_pinged) ||
                  ((now_us - g_last_ping_us) >= PING_CYCLE_US)))
        {
            /* Reached the next mark: fire */
            slot                     = (mark_deg - low_deg) / step_deg;
            p_points[slot].angle_deg = mark_deg;
            if (ultrasonic_ping_start())
            {
                b_pinging       = TRUE;
                g_b_ever_pinged = TRUE;
                g_last_ping_us  = now_us;
            }
            else
            {
                /* ECHO stuck high: record it and move on (NFR5-5) */
                p_points[slot].status      = ULTRASONIC_BUSY;
                p_points[slot].distance_mm = 0U;
                done++;
                mark_deg = b_up ? (mark_deg + step_deg) : (mark_deg - step_deg);
            }
        }
        else
        {
            /* gliding towards the next mark */
        }

        if (done < points)
        {
            tk_dly_tsk(SERVO_GLIDE_POLL_MS);
        }
    }
    return points;
}

/* How long the horn stays at each coarse angle after its ping, before
 * jumping on.  Jumping again straight after the ping looked rushed and
 * shook the robot; a short pause lets the horn and chassis go still and
 * makes each stop clearly visible.  [Tune on the robot] */
#define COARSE_DWELL_MS (500U)

/*
 * Coarse scan (FR5-1): jump to 50, 70, 90, 110 and 130 in turn
 * (SCAN_MIN_DEG to SCAN_MAX_DEG in COARSE_STEP_DEG steps).  At each: wait
 * for the horn to stop (servo_settle_ms), ping once, then stay
 * COARSE_DWELL_MS before the next jump.  Going in angle order keeps every
 * jump to 20 deg (only the first, from 90, is 40).  The points come out
 * in angle order.  No printing inside.
 * @return Number of points (COARSE_POINTS).
 */
static UW coarse_scan(scan_point_t * p_points)
{
    UW index;
    UW angle_deg;

    for (index = 0U; index < COARSE_POINTS; index++)
    {
        angle_deg = SCAN_MIN_DEG + (index * COARSE_STEP_DEG);
        servo_jump_to(angle_deg);

        p_points[index].angle_deg = angle_deg;
        p_points[index].status    = ultrasonic_measure_mm(
            ULTRASONIC_WINDOW_SCAN_MS, &p_points[index].distance_mm);

        tk_dly_tsk(COARSE_DWELL_MS);
    }
    return COARSE_POINTS;
}

/* Turn an ultrasonic status into text */
static const char * status_name(ultrasonic_status_t status)
{
    switch (status)
    {
        case ULTRASONIC_OK: return "ok";
        case ULTRASONIC_BUSY: return "busy";
        case ULTRASONIC_NO_ECHO: return "no-echo";
        case ULTRASONIC_TOO_NEAR: return "too-near";
        case ULTRASONIC_TOO_FAR: return "too-far";
        default: return "?";
    }
}

/* Turn a plan decision into text */
static const char * plan_name(plan_decision_t decision)
{
    switch (decision)
    {
        case PLAN_CONTINUE: return "CONTINUE";
        case PLAN_PASS_LEFT: return "PASS LEFT";
        case PLAN_PASS_RIGHT: return "PASS RIGHT";
        case PLAN_REVERSE: return "REVERSE";
        case PLAN_STOP: return "STOP (no path)";
        default: return "?";
    }
}

/*
 * Robot standing still: coarse scan, fine scan, profile and plan, with the
 * console printout.  Leaves the servo at 90.
 * @param  retries  Reverse-and-rescan attempts already made.
 * @param  p_plan   Out: the decision.
 * @param  p_profile Out: the obstacle profile (b_valid FALSE if none).
 */
static void scan_and_plan(UW retries, plan_t * p_plan,
                          obstacle_profile_t * p_profile)
{
    static scan_point_t coarse[COARSE_POINTS];
    static scan_point_t fine_raw[FINE_MAX_POINTS];
    static scan_point_t fine[FINE_MAX_POINTS];
    UW                  index;
    UW                  t0_us = rp2040_time_us();
    UW                  scan_ms;
    UW                  coarse_count;
    UW                  fine_count;
    UW                  fine_start;
    UW                  fine_end;

    /* Coarse (FR5-1): jump to the 5 angles, ping at each */
    coarse_count = coarse_scan(coarse);

    if (!fine_scan_window(coarse, coarse_count, &fine_start, &fine_end))
    {
        /* Nothing anymore, false alarm or obstacle moved */
        p_profile->b_valid = FALSE;
        servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
        p_plan->decision = PLAN_CONTINUE;
        p_plan->shift_mm = 0U;
        tm_printf((UB *)"S5: no obstacle in coarse scan\n");
        return;
    }

    /* Fine (FR5-2): glide to the low end of the detected span, then
     * smoothly across it, pinging every 5 deg
     * (e.g. hits at 90 and 110 -> 70, 75, ... 130) */
    fine_count = glide_and_ping(fine_start, fine_end, FINE_STEP_DEG, fine_raw);
    servo_glide_to(SERVO_ANGLE_CENTRE_DEG);

    scan_median3(fine_raw, fine, fine_count);
    (void)obstacle_profile(fine, fine_count, coarse, coarse_count, p_profile);
    obstacle_plan(p_profile, retries, p_plan);
    scan_ms = (rp2040_time_us() - t0_us) / 1000U;

    for (index = 0U; index < coarse_count; index++)
    {
        tm_printf((UB *)"S5: coarse %3u deg %4u  %s\n", coarse[index].angle_deg,
                  coarse[index].distance_mm, status_name(coarse[index].status));
    }
    tm_printf((UB *)"S5: fine %u..%u deg, %u points, scan %u ms\n", fine_start,
              fine_end, fine_count, scan_ms);
    for (index = 0U; index < fine_count; index++)
    {
        tm_printf((UB *)"S5: %3u deg  raw %4u  med %4u  %s\n",
                  fine[index].angle_deg, fine_raw[index].distance_mm,
                  fine[index].distance_mm, status_name(fine[index].status));
    }

    if (p_profile->b_valid)
    {
        tm_printf((UB *)"S5: closest %u mm at %u deg (x %d)\n",
                  p_profile->closest_mm, p_profile->closest_deg,
                  p_profile->closest_x_mm);
        tm_printf((UB *)"S5: centre y %d mm, width %u mm\n",
                  p_profile->centre_y_mm, p_profile->width_mm);
        tm_printf((UB *)"S5: clear L %u mm, R %u mm (9999 = open, 9998 = "
                        "not seen: the box runs past the scan)\n",
                  p_profile->clear_left_mm, p_profile->clear_right_mm);
    }
    else
    {
        tm_printf((UB *)"S5: no obstacle in fine scan\n");
    }

    tm_printf((UB *)"S5: PLAN %s, shift %u mm\n", plan_name(p_plan->decision),
              p_plan->shift_mm);
    if (p_plan->b_side_unseen)
    {
        tm_printf((UB *)"S5: WARNING: no side seen clear; passing on a side "
                        "the scan never saw past\n");
    }
}

/*
 * One forward leg, watching straight ahead while it drives: anything
 * closer than BYPASS_FRONT_STOP_MM (a valid reading) stops the robot
 * (HOLD) and fails the leg.
 * @return the reply bits (MOTION_DONE, or LINE_FOUND for a stop-on-line
 *         leg), or 0 on a failed or timed-out move, or something ahead.
 */
static UINT forward_watched(const bypass_step_t * p_step)
{
    UINT                wanted  = S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED;
    UINT                pattern = 0U;
    UW                  distance_mm;
    UW                  t0_us;
    ultrasonic_status_t status;

    if (p_step->b_stop_on_line)
    {
        wanted |= S5_EVT_LINE_FOUND;
    }

    /* Start the leg, but don't wait: watch ahead meanwhile */
    (void)tk_clr_flg(flg_s5_evt, ~wanted);
    s5_post_line(S5_REQ_MOVE, S5_MOVE_FWD, p_step->value,
                 p_step->b_stop_on_line, p_step->b_line_all);
    t0_us = rp2040_time_us();

    for (;;)
    {
        /* The controller may have halted (IMU, safety) and will never
         * reply */
        if ((rp2040_time_us() - t0_us) > ((UW)MOVE_TIMEOUT_MS * 1000U))
        {
            tm_printf((UB *)"S5: leg timed out\n");
            return 0U;
        }

        if (E_OK == tk_wai_flg(flg_s5_evt, wanted, TWF_ORW, &pattern, TMO_POL))
        {
            return ((pattern & S5_EVT_MOVE_FAILED) != 0U) ? 0U
                                                          : (pattern & wanted);
        }

        status = ultrasonic_measure_mm(ULTRASONIC_WINDOW_SCAN_MS, &distance_mm);
        if ((ULTRASONIC_OK == status) && (distance_mm < BYPASS_FRONT_STOP_MM))
        {
            tm_printf((UB *)"S5: something %u mm ahead on the bypass\n",
                      distance_mm);
            (void)s5_hold();
            return 0U;
        }

        tk_dly_tsk(MONITOR_GAP_MS); /* sensor needs >= 60 ms per cycle */
    }
}

/*
 * F2, the watched pass: drive on with the sensor turned towards the box
 * until it has been passed, then far enough for the robot's tail to clear
 * its far end (see "Watched pass" in obstacle_logic.h).  Up to two
 * stages: first bypass_pass_plan_cm() (where a box never seen ends it,
 * and which is always driven in full), then on towards PASS_MAX_CM while
 * the box is still beside.  Every side reading is logged.
 * @return TRUE once past; FALSE on a failed or timed-out move, or a box
 *         still beside at PASS_MAX_CM.
 */
static BOOL watch_pass(const plan_t *             p_plan,
                       const obstacle_profile_t * p_profile)
{
    side_pass_t         side      = {FALSE, 0U};
    const UW            gap_mm    = bypass_side_gap_mm(p_plan, p_profile);
    const UW            near_mm   = bypass_side_near_mm(gap_mm);
    UW                  stage_cm  = bypass_pass_plan_cm(p_profile);
    UW                  driven_cm = 0U; /* finished stages */
    UINT                pattern   = 0U;
    UW                  distance_mm;
    UW                  t0_us;
    UW                  extra_cm;
    ultrasonic_status_t status;
    BOOL                b_passed = FALSE;

    tm_printf((UB *)"S5: watching the box at %u deg (gap ~%u mm, beside "
                    "< %u mm), plan %u cm, up to %u cm\n",
              bypass_side_deg(p_plan->decision), gap_mm, near_mm, stage_cm,
              PASS_MAX_CM);
    servo_glide_to(bypass_side_deg(p_plan->decision));

    while (!b_passed)
    {
        /* Start this stage, but don't wait: watch the side meanwhile */
        (void)tk_clr_flg(flg_s5_evt,
                         ~(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED));
        s5_post(S5_REQ_MOVE, S5_MOVE_FWD, stage_cm, FALSE);
        t0_us = rp2040_time_us();

        for (;;)
        {
            /* The controller may have halted (IMU, safety) and will never
             * reply */
            if ((rp2040_time_us() - t0_us) > ((UW)MOVE_TIMEOUT_MS * 1000U))
            {
                tm_printf((UB *)"S5: watched leg timed out\n");
                return FALSE;
            }

            pattern = 0U;
            if (E_OK == tk_wai_flg(flg_s5_evt,
                                   S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED,
                                   TWF_ORW, &pattern, TMO_POL))
            {
                break; /* this stage ended */
            }

            status =
                ultrasonic_measure_mm(ULTRASONIC_WINDOW_SCAN_MS, &distance_mm);
            tm_printf((UB *)"S5: side %u mm %s\n", distance_mm,
                      status_name(status));
            if (!b_passed &&
                side_pass_update(&side, status, distance_mm, near_mm))
            {
                b_passed = TRUE; /* past the far end (by the side watch) */
                if (0U != driven_cm)
                {
                    break; /* beyond the plan: drive on from here */
                }
                /* Inside the planned distance: never stop before it */
                tm_printf((UB *)"S5: box looks passed inside the plan: "
                                "finishing the plan's %u cm first\n",
                          stage_cm);
            }

            tk_dly_tsk(MONITOR_GAP_MS); /* sensor needs >= 60 ms per cycle */
        }

        if (0U != (pattern & S5_EVT_MOVE_FAILED))
        {
            tm_printf((UB *)"S5: watched leg failed\n");
            return FALSE;
        }
        if (b_passed)
        {
            break;
        }

        driven_cm += stage_cm;
        if (!side.b_seen)
        {
            /* Too shallow to reach the beam: the plan's distance will do */
            tm_printf((UB *)"S5: box not seen in %u cm (shallow)\n", driven_cm);
            servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
            return TRUE;
        }
        if (driven_cm >= PASS_MAX_CM)
        {
            tm_printf((UB *)"S5: box still beside after %u cm: too long\n",
                      driven_cm);
            return FALSE;
        }

        /* Box still beside the sensor: carry on towards the limit */
        stage_cm = PASS_MAX_CM - driven_cm;
        tm_printf((UB *)"S5: box still beside, up to %u cm more\n", stage_cm);
    }

    /* Passed: drive on (a new request cuts the running move short) */
    extra_cm = bypass_after_clear_cm(gap_mm);
    tm_printf((UB *)"S5: box passed, %u cm on for the tail\n", extra_cm);
    if (S5_EVT_MOTION_DONE != s5_move(S5_MOVE_FWD, extra_cm, FALSE))
    {
        tm_printf((UB *)"S5: drive-on failed\n");
        return FALSE;
    }
    servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
    return TRUE;
}

/*
 * After T3 (facing the line, servo at 90): is the box still straight
 * ahead?  Two of three readings that are under 'limit_mm' OR too close to
 * measure (TOO_NEAR: the box is right against the sensor) say yes.  A
 * TOO_NEAR reading used to count as clear, and the robot drove on into a
 * box it was touching (2026-11-08).  No echo / too far / busy count as
 * clear.  Every reading is logged.
 */
static BOOL ahead_blocked(UW limit_mm)
{
    UW                  count = 0U;
    UW                  near  = 0U;
    UW                  distance_mm;
    ultrasonic_status_t status;

    for (count = 0U; count < 3U; count++)
    {
        status = ultrasonic_measure_mm(ULTRASONIC_WINDOW_SCAN_MS, &distance_mm);
        if (((ULTRASONIC_OK == status) && (distance_mm < limit_mm)) ||
            (ULTRASONIC_TOO_NEAR == status))
        {
            near++;
        }
        tm_printf((UB *)"S5: ahead %u mm %s\n", distance_mm,
                  status_name(status));
        tk_dly_tsk(MONITOR_GAP_MS); /* sensor needs >= 60 ms per cycle */
    }
    return (near >= 2U) ? TRUE : FALSE;
}

/* The opposite pivot (to undo T3) */
static UB opposite_turn(UB move)
{
    return (S5_MOVE_TURN_L == move) ? S5_MOVE_TURN_R : S5_MOVE_TURN_L;
}

/*
 * T3 has just turned the robot towards the line.  If the box is still
 * ahead, undo the turn, drive PROBE_STEP_CM on and turn again, up to
 * PROBE_TRIES times (the fallback when the side watch was fooled).
 * @return FALSE if still blocked after that, or a move failed.
 */
static BOOL clear_ahead(const bypass_step_t * p_turn, UW limit_mm)
{
    bypass_step_t on = {S5_MOVE_FWD, PROBE_STEP_CM, FALSE, FALSE, FALSE, FALSE};
    UW            tries = 0U;

    while (ahead_blocked(limit_mm))
    {
        if (tries >= PROBE_TRIES)
        {
            tm_printf((UB *)"S5: box still ahead after %u tries\n", tries);
            return FALSE;
        }
        tries++;
        tm_printf((UB *)"S5: box still ahead (< %u mm): back, %u cm on, "
                        "turn again (%u/%u)\n",
                  limit_mm, PROBE_STEP_CM, tries, PROBE_TRIES);
        if ((S5_EVT_MOTION_DONE !=
             s5_move(opposite_turn(p_turn->move), p_turn->value, FALSE)) ||
            (S5_EVT_MOTION_DONE != forward_watched(&on)) ||
            (S5_EVT_MOTION_DONE != s5_move(p_turn->move, p_turn->value, FALSE)))
        {
            tm_printf((UB *)"S5: retry move failed\n");
            return FALSE;
        }
    }
    return TRUE;
}

/* Short name of a bypass move, for the log */
static const char * move_name(UB move)
{
    switch (move)
    {
        case S5_MOVE_FWD: return "forward";
        case S5_MOVE_TURN_L: return "turn L";
        case S5_MOVE_TURN_R: return "turn R";
        default: return "?";
    }
}

/*
 * Drive round the box on the turn-and-box path and back onto the line
 * behind it (FR5-4, FR5-5).  See the move list in obstacle_logic.h.  The
 * robot is at rest, servo at 90 (it turns to the box's side only during
 * F2).  Ends facing along the line; the caller then hands back to line
 * following.
 * @return FALSE on any failure (caller stops safely).
 */
static BOOL bypass_obstacle(const plan_t *             p_plan,
                            const obstacle_profile_t * p_profile)
{
    bypass_step_t step;
    UW            index;
    UINT          result;

    /* The first pivot swings the robot's front corner forward: make sure
     * the box is not right against it */
    if (p_profile->closest_x_mm < (INT)BYPASS_MIN_GAP_MM)
    {
        tm_printf((UB *)"S5: box %d mm ahead: too close to turn\n",
                  p_profile->closest_x_mm);
        return FALSE;
    }

    for (index = 0U; index < BYPASS_STEPS; index++)
    {
        if (!bypass_step(p_plan, index, &step))
        {
            return FALSE;
        }
        tm_printf((UB *)"S5: step %u/%u: %s %u %s%s\n", index + 1U,
                  BYPASS_STEPS, move_name(step.move), step.value,
                  (S5_MOVE_FWD == step.move) ? "cm" : "deg",
                  step.b_watch_side     ? " (at most, watching the box)"
                  : step.b_stop_on_line ? " (until the line)"
                                        : "");

        if (step.b_watch_side)
        {
            result = watch_pass(p_plan, p_profile) ? S5_EVT_MOTION_DONE : 0U;
        }
        else if (S5_MOVE_FWD == step.move)
        {
            result = forward_watched(&step);
        }
        else
        {
            result = s5_move(step.move, step.value, step.b_stop_on_line);
            if ((S5_EVT_MOTION_DONE != result) && (S5_EVT_LINE_FOUND != result))
            {
                result = 0U;
            }
        }

        if (0U == result)
        {
            tm_printf((UB *)"S5: step %u failed\n", index + 1U);
            return FALSE;
        }
        if (step.b_check_ahead &&
            !clear_ahead(
                &step, bypass_probe_mm(bypass_side_gap_mm(p_plan, p_profile))))
        {
            return FALSE;
        }
        if (step.b_stop_on_line)
        {
            if (0U == (result & S5_EVT_LINE_FOUND))
            {
                tm_printf((UB *)"S5: LINE LOST after %u %s (NFR5-4)\n",
                          step.value,
                          (S5_MOVE_FWD == step.move) ? "cm" : "deg");
                return FALSE;
            }
            tm_printf((UB *)"S5: line found\n");
        }
    }
    return TRUE;
}

/* Main obstacle task: monitor, hold, scan, plan, then resume or stop */
EXPORT void obstacle_task(INT stacd, void * exinf)
{
    s5_state_t          state   = S5_MONITORING;
    UB                  history = 0U;
    UW                  distance_mm;
    ultrasonic_status_t status;
    plan_t              plan;
    obstacle_profile_t  profile;

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    /* Start-up hold: the VC won't arm while S5 asks for HOLD.  The servo's
     * first swing shakes the robot hard enough for S4 to report COLLISION
     * or TILT, which the VC latches once armed (seen 2026-11-05). */
    s5_post(S5_REQ_HOLD, S5_MOVE_FWD, 0U, FALSE);

    ultrasonic_init();

    /* Also keep the swing out of the IMU baseline average: a bad
     * baseline later reads as TILT. */
    wait_for_imu_calibration();
    servo_init();
    tk_dly_tsk(STARTUP_SETTLE_MS);

    /* Servo parked and still: let the VC arm and follow the line */
    s5_post(S5_REQ_NONE, S5_MOVE_FWD, 0U, FALSE);
    tm_printf((UB *)"S5: MONITORING\n");

    /* A task must never return: loop forever */
    for (;;)
    {
        switch (state)
        {
            case S5_MONITORING:
                if (!s5_detect_allowed())
                {
                    history = 0U; /* old readings no longer count */
                    tk_dly_tsk(IDLE_POLL_MS);
                    break;
                }

                status = ultrasonic_measure_mm(ULTRASONIC_WINDOW_SCAN_MS,
                                               &distance_mm);

                if (obstacle_filter_update(&history, status, distance_mm))
                {
#if S5_HAS_VC
                    tm_printf((UB *)"S5: OBSTACLE at %u mm -> HOLD\n",
                              distance_mm);
                    state = S5_HOLDING;
#else
                    tm_printf((UB *)"S5: OBSTACLE at %u mm -> SCANNING "
                                    "(no VC in this build: robot not "
                                    "stopped)\n",
                              distance_mm);
                    state = S5_SCANNING;
#endif
                }
                else
                {
                    /* Sensor needs >= 60 ms between pings (datasheet) */
                    tk_dly_tsk(MONITOR_GAP_MS);
                }
                break;

            case S5_HOLDING:
                if (s5_hold())
                {
                    state = S5_SCANNING;
                }
                else
                {
                    tm_printf((UB *)"S5: VC did not stop\n");
                    state = S5_SAFE_STOP;
                }
                break;

            case S5_SCANNING:
                /* Retries = MAX_RETRIES: S2 can't reverse, so "blocked"
                 * means PLAN_STOP (NO PATH) at once */
                scan_and_plan(MAX_RETRIES, &plan, &profile);

#if !S5_HAS_VC
                /* Bench behaviour: nobody to ask for moves, so just report
                 * the plan and keep watching */
                tk_dly_tsk(RESUME_QUIET_MS);
                history = 0U; /* forget readings from before the scan */
                tm_printf((UB *)"S5: MONITORING\n");
                state = S5_MONITORING;
                break;
#endif

                if (PLAN_CONTINUE == plan.decision)
                {
                    /* Not in the way: hand control back */
                    s5_post(S5_REQ_RESUME, S5_MOVE_FWD, 0U, FALSE);
                    tm_printf((UB *)"S5: RESUME line following\n");
                    tk_dly_tsk(RESUME_QUIET_MS);
                    history = 0U; /* forget readings from before */
                    tm_printf((UB *)"S5: MONITORING\n");
                    state = S5_MONITORING;
                }
                else if ((PLAN_PASS_LEFT == plan.decision) ||
                         (PLAN_PASS_RIGHT == plan.decision))
                {
                    state = S5_BYPASSING;
                }
                else
                {
                    /* REVERSE (S2 has no reverse) or STOP: NO PATH */
                    tm_printf((UB *)"S5: NO PATH\n");
                    state = S5_SAFE_STOP;
                }
                break;

            case S5_BYPASSING:
                tm_printf((UB *)"S5: BYPASS %s\n", plan_name(plan.decision));
                if (!bypass_obstacle(&plan, &profile))
                {
                    servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
                    state = S5_SAFE_STOP;
                    break;
                }

                /* Back on the line: hand control back to line following */
                s5_post(S5_REQ_RESUME, S5_MOVE_FWD, 0U, FALSE);
                tm_printf((UB *)"S5: RESUME line following\n");
                tk_dly_tsk(RESUME_QUIET_MS);
                history = 0U; /* forget readings from before */
                tm_printf((UB *)"S5: MONITORING\n");
                state = S5_MONITORING;
                break;

            case S5_SAFE_STOP:
                s5_post(S5_REQ_SAFE_STOP, S5_MOVE_FWD, 0U, FALSE);
                servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
                tm_printf((UB *)"S5: SAFE STOP (reboot to re-arm)\n");
                (void)tk_slp_tsk(TMO_FEVR); /* stay stopped */
                break;

            default:
                /* unknown state: should never happen; stop safely */
                state = S5_SAFE_STOP;
                break;
        }
    }
}

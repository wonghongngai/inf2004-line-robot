#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_logic.h"

#define MONITOR_GAP_MS (30U)   /* + ~30 ms ping = one ping per ~70-80 ms */
#define RESUME_MS      (3000U) /* bench only: pause after a scan        */

typedef enum
{
    S5_MONITORING = 0, /* servo forward, pinging, watching for obstacles */
    S5_SCANNING        /* obstacle confirmed: coarse jumps + fine glide  */
} s5_state_t;

#define PING_CYCLE_US  (60000U) /* sensor: >= 60 ms between pings */
#define PING_WINDOW_US (ULTRASONIC_WINDOW_SCAN_MS * 1000U)

/* When the last ping was fired, across passes: the sensor needs
 * PING_CYCLE_US between pings even when one pass ends and the next
 * starts at the same angle. */
static UW   g_last_ping_us   = 0U;
static BOOL g_b_ever_pinged  = FALSE;

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
    const UW   points  = (((b_up ? (to_deg - from_deg) : (from_deg - to_deg))) /
                         step_deg) + 1U;
    UW   mark_deg  = from_deg;
    UW   done      = 0U;
    UW   slot      = 0U;
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
                mark_deg = b_up ? (mark_deg + step_deg)
                                : (mark_deg - step_deg);
            }
        }
        else if (mark_reached(mark_deg, b_up) &&
                 ((!g_b_ever_pinged) ||
                  ((now_us - g_last_ping_us) >= PING_CYCLE_US)))
        {
            /* Reached the next mark: fire */
            slot                      = (mark_deg - low_deg) / step_deg;
            p_points[slot].angle_deg  = mark_deg;
            if (ultrasonic_ping_start())
            {
                b_pinging        = TRUE;
                g_b_ever_pinged  = TRUE;
                g_last_ping_us   = now_us;
            }
            else
            {
                /* ECHO stuck high: record it and move on (NFR5-5) */
                p_points[slot].status      = ULTRASONIC_BUSY;
                p_points[slot].distance_mm = 0U;
                done++;
                mark_deg = b_up ? (mark_deg + step_deg)
                                : (mark_deg - step_deg);
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

/* Main obstacle task: monitor, scan, profile and plan */
EXPORT void obstacle_task(INT stacd, void * exinf)
{
    s5_state_t          state   = S5_MONITORING;
    UB                  history = 0U;
    static scan_point_t coarse[COARSE_POINTS];
    static scan_point_t fine_raw[FINE_MAX_POINTS];
    static scan_point_t fine[FINE_MAX_POINTS];
    UW                  distance_mm;
    UW                  index;
    UW                  t0_us;
    UW                  scan_ms;
    UW                  coarse_count;
    UW                  fine_count;
    UW                  fine_start;
    UW                  fine_end;
    obstacle_profile_t  profile;
    ultrasonic_status_t status;
    plan_t              plan;
    UW                  retries = 0U;

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    ultrasonic_init();
    servo_init();

    tm_printf((UB *)"S5: MONITORING\n");

    /* A task must never return: loop forever */
    for (;;)
    {
        switch (state)
        {
            case S5_MONITORING:
                status = ultrasonic_measure_mm(ULTRASONIC_WINDOW_SCAN_MS,
                                               &distance_mm);

                if (obstacle_filter_update(&history, status, distance_mm))
                {
                    tm_printf((UB *)"S5: OBSTACLE at %u mm -> SCANNING\n",
                              distance_mm);
                    state = S5_SCANNING;
                }
                else
                {
                    /* Sensor needs >= 60 ms between pings (datasheet) */
                    tk_dly_tsk(MONITOR_GAP_MS);
                }
                break;

            case S5_SCANNING:
                t0_us = rp2040_time_us();

                /* Coarse (FR5-1): jump to the 5 angles, ping at each */
                coarse_count = coarse_scan(coarse);

                if (!fine_scan_window(coarse, coarse_count, &fine_start,
                                      &fine_end))
                {
                    /* Nothing anymore, false alarm or obstacle moved */
                    servo_glide_to(SERVO_ANGLE_CENTRE_DEG);
                    plan.decision = PLAN_CONTINUE;
                    plan.shift_mm = 0U;
                    tm_printf((UB *)"S5: no obstacle in coarse scan\n");
                }
                else
                {
                    /* Fine (FR5-2): glide to the low end of the detected
                     * span, then smoothly across it, pinging every 5 deg
                     * (e.g. hits at 70 and 90 -> 70, 75, ... 90) */
                    fine_count = glide_and_ping(fine_start, fine_end,
                                                FINE_STEP_DEG, fine_raw);
                    servo_glide_to(SERVO_ANGLE_CENTRE_DEG);

                    scan_median3(fine_raw, fine, fine_count);
                    (void)obstacle_profile(fine, fine_count, coarse,
                                           coarse_count, &profile);
                    obstacle_plan(&profile, retries, &plan);
                    scan_ms = (rp2040_time_us() - t0_us) / 1000U;

                    for (index = 0U; index < coarse_count; index++)
                    {
                        tm_printf((UB *)"S5: coarse %3u deg %4u  %s\n",
                                  coarse[index].angle_deg,
                                  coarse[index].distance_mm,
                                  status_name(coarse[index].status));
                    }
                    tm_printf((UB *)"S5: fine %u..%u deg, %u points, "
                                    "scan %u ms\n",
                              fine_start, fine_end, fine_count, scan_ms);
                    for (index = 0U; index < fine_count; index++)
                    {
                        tm_printf((UB *)"S5: %3u deg  raw %4u  med %4u  %s\n",
                                  fine[index].angle_deg,
                                  fine_raw[index].distance_mm,
                                  fine[index].distance_mm,
                                  status_name(fine[index].status));
                    }

                    if (profile.b_valid)
                    {
                        tm_printf((UB *)"S5: closest %u mm at %u deg (x %d)\n",
                                  profile.closest_mm, profile.closest_deg,
                                  profile.closest_x_mm);
                        tm_printf((UB *)"S5: centre y %d mm, width %u mm\n",
                                  profile.centre_y_mm, profile.width_mm);
                        tm_printf((UB *)"S5: clear L %u mm, R %u mm "
                                        "(9999 = open)\n",
                                  profile.clear_left_mm,
                                  profile.clear_right_mm);
                    }
                    else
                    {
                        tm_printf((UB *)"S5: no obstacle in fine scan\n");
                    }

                    tm_printf((UB *)"S5: PLAN %s, shift %u mm\n",
                              plan_name(plan.decision), plan.shift_mm);
                }

                /* Bench stand-in for the moves (Step 14 asks Buddy 2) */
                tk_dly_tsk(RESUME_MS);
                history = 0U; /* forget readings from before the scan */

                if (PLAN_REVERSE == plan.decision)
                {
                    retries++;
                    tm_printf((UB *)"S5: (would reverse) rescan, retry %u\n",
                              retries);
                    state = S5_SCANNING; /* scan again from the new spot */
                }
                else
                {
                    if (PLAN_STOP == plan.decision)
                    {
                        tm_printf((UB *)"S5: NO PATH (would stay stopped)\n");
                    }
                    retries = 0U; /* next obstacle starts fresh */
                    tm_printf((UB *)"S5: MONITORING\n");
                    state = S5_MONITORING;
                }
                break;

            default:
                /* unknown state: should never happen; recover */
                state = S5_MONITORING;
                break;
        }
    }
}
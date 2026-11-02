#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_logic.h"

#define COARSE_COUNT   (5U)
#define MONITOR_GAP_MS (30U)   /* + ~30 ms ping = one ping per ~70-80 ms */
#define RESUME_MS      (3000U) /* bench only: pause after a scan        */

typedef enum
{
    S5_MONITORING = 0, /* servo forward, pinging, watching for obstacles */
    S5_SCANNING        /* obstacle confirmed: coarse scan                */
} s5_state_t;

/* Starts at 90 (where the servo is parked) so the forward
 * reading comes first; total travel 180 deg. */
static const UW g_coarse_angles_deg[COARSE_COUNT] = {90U, 60U, 30U, 120U, 150U};

/* One coarse scan: move, settle, ping at each angle.  No printing
 * inside: printing takes time and would spoil the timing. */
static void coarse_scan(scan_point_t * p_points)
{
    UW index;

    for (index = 0U; index < COARSE_COUNT; index++)
    {
        servo_move_and_settle(g_coarse_angles_deg[index]);

        p_points[index].angle_deg = g_coarse_angles_deg[index];
        p_points[index].status    = ultrasonic_measure_mm(
            ULTRASONIC_WINDOW_SCAN_MS, &p_points[index].distance_mm);
    }
}

static UW fine_scan(UW start_deg, UW end_deg, scan_point_t * p_points)
{
    UW angle_deg;
    UW count = 0U;

    for (angle_deg = start_deg;
         (angle_deg <= end_deg) && (count < FINE_MAX_POINTS);
         angle_deg += FINE_STEP_DEG)
    {
        servo_move_and_settle(angle_deg);

        p_points[count].angle_deg = angle_deg;
        p_points[count].status    = ultrasonic_measure_mm(
            ULTRASONIC_WINDOW_SCAN_MS, &p_points[count].distance_mm);
        count++;
    }

    return count;
}

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

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    s5_state_t          state   = S5_MONITORING;
    UB                  history = 0U;
    static scan_point_t points[COARSE_COUNT];
    static scan_point_t fine_raw[FINE_MAX_POINTS];
    static scan_point_t fine[FINE_MAX_POINTS];
    UW                  distance_mm;
    UW                  index;
    UW                  t0_us;
    UW                  scan_ms;
    UW                  fine_count;
    UW                  start_deg;
    UW                  end_deg;
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
                coarse_scan(points);

                if (!fine_scan_window(points, COARSE_COUNT, &start_deg,
                                      &end_deg))
                {
                    /* Nothing anymore, false alarm or obstacle moved */
                    plan.decision = PLAN_CONTINUE;
                    servo_move_and_settle(SERVO_ANGLE_CENTRE_DEG);
                    tm_printf((UB *)"S5: no obstacle in coarse scan\n");
                }
                else
                {
                    fine_count = fine_scan(start_deg, end_deg, fine_raw);
                    servo_move_and_settle(SERVO_ANGLE_CENTRE_DEG);

                    scan_median3(fine_raw, fine, fine_count);
                    (void)obstacle_profile(fine, fine_count, points,
                                           COARSE_COUNT, &profile);
                    obstacle_plan(&profile, retries, &plan);
                    scan_ms = (rp2040_time_us() - t0_us) / 1000U;

                    tm_printf((UB *)"S5: fine %u..%u deg, %u points\n",
                              start_deg, end_deg, fine_count);

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
                    tm_printf((UB *)"S5: detect->decision %u ms "
                                    "(NFR5-3: <= 2000)\n",
                              scan_ms);
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
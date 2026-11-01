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

typedef struct
{
    UW                  angle_deg;
    UW                  distance_mm;
    ultrasonic_status_t status;
} scan_point_t;

/* One coarse scan: move, settle, ping at each angle.  No printing
 * inside: printing takes time and would spoil the timing (NFR5-2). */
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

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    s5_state_t          state   = S5_MONITORING;
    UB                  history = 0U;
    scan_point_t        points[COARSE_COUNT];
    ultrasonic_status_t status;
    UW                  distance_mm;
    UW                  index;
    UW                  t0_us;
    UW                  scan_ms;

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
                    tk_dly_tsk(
                        MONITOR_GAP_MS); /* sensor needs >= 60ms as stated in datasheet */
                }
                break;

            case S5_SCANNING:
                t0_us = rp2040_time_us();
                coarse_scan(points);
                scan_ms = (rp2040_time_us() - t0_us) / 1000U;
                servo_move_and_settle(SERVO_ANGLE_CENTRE_DEG);

                for (index = 0U; index < COARSE_COUNT; index++)
                {
                    tm_printf((UB *)"S5: %3u deg %4u mm %s\n",
                              points[index].angle_deg,
                              points[index].distance_mm,
                              status_name(points[index].status));
                }
                tm_printf((UB *)"S5: coarse scan %u ms\n", scan_ms);

                /* Bench stand in */
                tk_dly_tsk(RESUME_MS);
                history = 0U; /* forget readings before scan */
                tm_printf((UB *)"S5: MONITORING\n");
                state = S5_MONITORING;
                break;

            default:
                /* unknown state: should never happen; recover */
                state = S5_MONITORING;
                break;
        }
    }
}
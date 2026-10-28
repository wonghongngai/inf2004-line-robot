#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#define COARSE_COUNT (5U)

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
        p_points[index].status = ultrasonic_measure_mm(
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
    scan_point_t points[COARSE_COUNT];
    UW           index;
    UW           t0_us;
    UW           scan_ms;

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    ultrasonic_init();
    servo_init();

    /* A task must never return: loop forever */
    for (;;)
    {
        t0_us = rp2040_time_us();
        coarse_scan(points);
        scan_ms = (rp2040_time_us() - t0_us) / 1000U;

        servo_move_and_settle(SERVO_ANGLE_CENTRE_DEG); /* park forward */

        for (index = 0U; index < COARSE_COUNT; index++)
        {
            tm_printf((UB *)"S5: %3u deg  %4u mm  %s\n",
                      points[index].angle_deg, points[index].distance_mm,
                      status_name(points[index].status));
        }
        tm_printf((UB *)"S5: coarse scan %u ms (NFR5-2: <= 600)\n\n", scan_ms);

        tk_dly_tsk(2000); /* pause to read the console */
    }
}
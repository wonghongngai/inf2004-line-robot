/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "obstacle_logic.h"
#include "../test_report.h"

#if APP_TEST

#define SWEEP_DDEG      (1200U) /* a 120 deg glide (maths only)    */
#define PWM_PERIOD_MS   (20U)   /* the servo reads one pulse per 20 ms */
#define DEAD_BAND_DDEG  (9U)    /* SG90 10 us dead band ~ 0.9 deg    */
#define MAX_MOVE_DDEG   (17U)   /* cruise 80 deg/s = 1.6 deg/period  */

/*
 * Sample a glide once per PWM period, as the servo sees it.  Reports
 * whether every period's change is above the dead band (no stick-and-
 * jump) and no faster than cruise, it never goes backwards, and it ends
 * exactly on the distance.  The final period may be a short remainder.
 */
static BOOL glide_is_smooth(UW distance_ddeg)
{
    UW   total_ms = servo_profile_total_ms(distance_ddeg);
    UW   last     = servo_profile_ddeg(distance_ddeg, 0U);
    UW   now;
    UW   t_ms;
    BOOL b_smooth = (0U == last) ? TRUE : FALSE;

    for (t_ms = PWM_PERIOD_MS; t_ms < total_ms; t_ms += PWM_PERIOD_MS)
    {
        now = servo_profile_ddeg(distance_ddeg, t_ms);
        if ((now < last) || ((now - last) < DEAD_BAND_DDEG) ||
            ((now - last) > MAX_MOVE_DDEG))
        {
            b_smooth = FALSE;
        }
        last = now;
    }

    return (b_smooth &&
            (distance_ddeg == servo_profile_ddeg(distance_ddeg, total_ms)))
               ? TRUE
               : FALSE;
}

/* Fill a coarse scan (50/70/90/110/130), all NO_ECHO */
static void make_coarse(scan_point_t * p_points)
{
    UW index;

    for (index = 0U; index < COARSE_POINTS; index++)
    {
        p_points[index].angle_deg   = SCAN_MIN_DEG + (index * COARSE_STEP_DEG);
        p_points[index].distance_mm = 0U;
        p_points[index].status      = ULTRASONIC_NO_ECHO;
    }
}

/* Make the coarse point at 'angle_deg' an OK reading of 'distance_mm' */
static void set_hit(scan_point_t * p_points, UW angle_deg, UW distance_mm)
{
    UW index = (angle_deg - SCAN_MIN_DEG) / COARSE_STEP_DEG;

    p_points[index].status      = ULTRASONIC_OK;
    p_points[index].distance_mm = distance_mm;
}

/* TRUE if fine_scan_window() gives exactly start..end for this scan */
static BOOL window_is(const scan_point_t * p_coarse, UW start_deg,
                      UW end_deg)
{
    UW got_start = 0U;
    UW got_end   = 0U;

    return (fine_scan_window(p_coarse, COARSE_POINTS, &got_start, &got_end) &&
            (start_deg == got_start) && (end_deg == got_end))
               ? TRUE
               : FALSE;
}

/* Run subsystem 5's self-tests */
EXPORT void subsystem5_test(void)
{
    scan_point_t coarse[COARSE_POINTS];
    UW           start_deg = 0U;
    UW           end_deg   = 0U;

    TEST_INIT();

    /* S5-U21: time-based glide profile */
    TEST_CHECK("S5-U21", (1525U == servo_profile_total_ms(SWEEP_DDEG)),
               "120 deg glide takes 1525 ms (ramp 100 + cruise 1325 + ramp 100)");
    TEST_CHECK("S5-U21", ((0U == servo_profile_ddeg(SWEEP_DDEG, 0U)) &&
                          (70U == servo_profile_ddeg(SWEEP_DDEG, 100U)) &&
                          (600U == servo_profile_ddeg(SWEEP_DDEG, 762U)) &&
                          (SWEEP_DDEG == servo_profile_ddeg(SWEEP_DDEG, 1525U)) &&
                          (SWEEP_DDEG == servo_profile_ddeg(SWEEP_DDEG, 5000U))),
               "starts at 0, 7 deg after the ramp, half way at mid-time, then stays at the end");
    TEST_CHECK("S5-U21", glide_is_smooth(SWEEP_DDEG),
               "120 deg: every 20 ms pulse moves 0.9-1.7 deg, never back, lands exactly");
    TEST_CHECK("S5-U21", glide_is_smooth(600U),
               "60 deg glide is smooth too");
    TEST_CHECK("S5-U21", ((167U == servo_profile_total_ms(100U)) &&
                          (50U == servo_profile_ddeg(100U, 83U)) &&
                          glide_is_smooth(100U)),
               "short 10 deg move runs at the start speed throughout");
    TEST_CHECK("S5-U21", ((0U == servo_profile_total_ms(0U)) &&
                          (0U == servo_profile_ddeg(0U, 50U))),
               "zero-length move takes no time");

    /* S5-U22: pulse width in tenths of a degree */
    TEST_CHECK("S5-U22", (1530U == servo_ddeg_to_pulse_us(900U)),
               "90.0 deg = 1530 us (forward)");
    TEST_CHECK("S5-U22", ((850U == servo_ddeg_to_pulse_us(300U)) &&
                          (2210U == servo_ddeg_to_pulse_us(1500U))),
               "30.0 / 150.0 deg = 850 / 2210 us");
    TEST_CHECK("S5-U22", (1536U == servo_ddeg_to_pulse_us(905U)),
               "90.5 deg = 1536 us: sub-degree steps change the pulse");
    TEST_CHECK("S5-U22", (servo_ddeg_to_pulse_us(100U) == servo_ddeg_to_pulse_us(200U)),
               "below 20 deg is clamped");
    TEST_CHECK("S5-U22", (servo_angle_to_pulse_us(120U) == servo_ddeg_to_pulse_us(1200U)),
               "whole-degree and ddeg conversions agree");

    /* S5-U28: fine-scan angles from the coarse scan */
    make_coarse(coarse);
    TEST_CHECK("S5-U28", ((5U == COARSE_POINTS) && (50U == coarse[0].angle_deg) &&
                          (70U == coarse[1].angle_deg) && (90U == coarse[2].angle_deg) &&
                          (110U == coarse[3].angle_deg) && (130U == coarse[4].angle_deg)),
               "coarse scan is 50 / 70 / 90 / 110 / 130");
    make_coarse(coarse);
    set_hit(coarse, 70U, 400U);
    set_hit(coarse, 90U, 350U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 90U),
               "hits at 70 and 90: fine 70..90");
    make_coarse(coarse);
    set_hit(coarse, 70U, 400U);
    set_hit(coarse, 90U, 350U);
    set_hit(coarse, 110U, 380U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 110U),
               "hits at 70, 90 and 110: fine 70..110");
    make_coarse(coarse);
    set_hit(coarse, 70U, 400U);
    set_hit(coarse, 110U, 350U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 110U),
               "hits at 70 and 110 (90 clear): fine 70..110");
    make_coarse(coarse);
    set_hit(coarse, 90U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 110U),
               "hit at 90 only: fine 70..110 (to both neighbours)");
    make_coarse(coarse);
    set_hit(coarse, 50U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 50U, 70U),
               "hit at 50 only: fine 50..70 (limited at 50)");
    make_coarse(coarse);
    set_hit(coarse, 130U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 110U, 130U),
               "hit at 130 only: fine 110..130 (limited at 130)");
    make_coarse(coarse);
    TEST_CHECK("S5-U28",
               !fine_scan_window(coarse, COARSE_POINTS, &start_deg, &end_deg),
               "no hits (all NO_ECHO): no fine scan");
    set_hit(coarse, 90U, 700U);
    TEST_CHECK("S5-U28",
               !fine_scan_window(coarse, COARSE_POINTS, &start_deg, &end_deg),
               "OK at 700 mm is beyond D_PROFILE_MM: not a hit");
    TEST_CHECK("S5-U28",
               (!fine_scan_window(NULL, COARSE_POINTS, &start_deg, &end_deg) &&
                !fine_scan_window(coarse, COARSE_POINTS, NULL, &end_deg) &&
                !fine_scan_window(coarse, COARSE_POINTS, &start_deg, NULL)),
               "NULL pointer gives FALSE");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

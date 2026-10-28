/*
 * subsystem4_test.c
 * On-target tests for Buddy 4. Built only with: make TEST=1
 *
 * These tests do not access I2C directly because imu_task is the owning task.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
    IMU_DATA before;
    IMU_DATA after;
    ER er;

    TEST_INIT();

    er = subsystem4_get_latest(&before);
    TEST_CHECK("S4-U01",
               er == E_OK,
               "latest IMU snapshot can be read safely");

    /* Allow enough time for 1 s calibration plus normal samples. */
    tk_dly_tsk(1500);

    er = subsystem4_get_latest(&after);
    TEST_CHECK("S4-U02",
               er == E_OK && after.sensor_ok,
               "LSM303DLHC is responding through Buddy 4 task");

    TEST_CHECK("S4-U03",
               after.calibrated,
               "stationary IMU calibration completes");

    TEST_CHECK("S4-U04",
               after.sample_count > before.sample_count,
               "20 ms IMU task continues producing samples");

    TEST_CHECK("S4-U05",
               after.last_cycle_ms <= S4_SAMPLE_PERIOD_MS,
               "IMU sensing and processing fits within 20 ms cycle");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

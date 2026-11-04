/*
 * subsystem4_test.c
 * On-target tests for Buddy 4. Build with: make TEST=1 -j8
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
    IMU_DATA before;
    IMU_DATA after;
    IMU_PACKED_MSG packed;
    ER er;

    TEST_INIT();

    er = subsystem4_get_latest(&before);
    TEST_CHECK("S4-U01",
               er == E_OK,
               "latest IMU snapshot can be read safely");

    /* Allow 1 s calibration plus normal samples. */
    tk_dly_tsk(1500);

    er = subsystem4_get_latest(&after);
    TEST_CHECK("S4-U02",
               er == E_OK && after.sensor_ok,
               "LSM303DLHC responds through Buddy 4 task");

    TEST_CHECK("S4-U03",
               after.calibrated,
               "stationary IMU calibration completes");

    TEST_CHECK("S4-U04",
               after.sample_count > before.sample_count,
               "IMU task continues producing samples");

    /* Timing evidence: count overruns instead of spamming serial warnings. */
    TEST_CHECK("S4-U05",
               after.last_cycle_ms <= S4_SAMPLE_PERIOD_MS,
               "latest IMU sensing/processing cycle fits within 20 ms");

    er = subsystem4_get_packed(&packed);
    TEST_CHECK("S4-U06",
               er == E_OK && sizeof(IMU_PACKED_MSG) == 4U,
               "compact Buddy 4 output is exactly 4 bytes");

    TEST_CHECK("S4-U07",
               packed.ramp_height_mm <= S4_MAX_RAMP_HEIGHT_MM,
               "8-bit ramp-height field remains within its valid range");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*
 * Raised-wheel, open-loop hardware test using the existing motor driver.
 * Runs instead of motion_task, once per boot. No calibration or PID used.
 * Motor/encoder initialisation is performed by subsystem2_init().
 */
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#if S2_MOTOR_HW_TEST

#if APP_ROBOT_INTEGRATED
#include "../subsystem3/line_service.h"
#include "../subsystem4/subsystem4.h"

/* Sample diagnostics only with motors OFF. IR decisions remain paused;
   these raised-wheel pulses do not use autonomous navigation/safety logic. */
static void report_sensors(void)
{
    line_barcode_msg_t line    = {{0u, 0u, 0u}};
    IMU_DATA           imu     = {0};
    ER                 line_er = line_service_snapshot(&line);
    ER                 imu_er  = subsystem4_get_latest(&imu);
    LOG_INFO(TAG_S2_HWTEST, "S3 IR heartbeat=%d bits=%u state=%u (paused)",
             line_er, (UINT)(line.data[0] & 7u), (UINT)(line.data[1] & 7u));
    LOG_INFO(TAG_S2_HWTEST,
             "S4 IMU read=%d sensor_ok=%d calibrated=%d samples=%lu errors=%lu",
             imu_er, imu.sensor_ok, imu.calibrated,
             (unsigned long)imu.sample_count, (unsigned long)imu.read_errors);
}
#endif

/* Poll while waiting, so encoder edges can be observed during each stage. */
static bool wait_and_poll(uint32_t duration_ms)
{
    SYSTIM start;
    SYSTIM now;

    if (tk_get_tim(&start) < E_OK)
    {
        return false;
    }
    do
    {
        drv_encoder_poll();
        if (tk_dly_tsk(S2_ENCODER_POLL_MS) < E_OK || tk_get_tim(&now) < E_OK)
        {
            return false;
        }
    } while ((uint32_t)(now.lo - start.lo) < duration_ms);
    return true;
}

static bool run_stage(const char * p_name, float left, float right)
{
    int32_t           left_ticks;
    int32_t           right_ticks;
    bool              b_waited;
    MOTOR_DIAGNOSTICS diag;
    ER                check;

    drv_encoder_reset();
    /* Print before applying power; stop before printing the result. */
    LOG_INFO(TAG_S2_HWTEST, "%s: %u ms", p_name, (UINT)S2_HW_TEST_RUN_MS);
    drv_motor_set(left, right);
    check    = drv_motor_diagnostics(&diag);
    b_waited = check == E_OK && wait_and_poll(S2_HW_TEST_RUN_MS);
    drv_motor_stop();
    drv_encoder_get_counts(&left_ticks, &right_ticks);
    LOG_INFO(TAG_S2_HWTEST, "STOP; ticks left=%ld right=%ld", (long)left_ticks,
             (long)right_ticks);
    LOG_INFO(TAG_S2_HWTEST, "PWM check=%d counting=%d/%d compare=%lu/%lu",
             check, diag.left_counting, diag.right_counting,
             (unsigned long)diag.left_compare,
             (unsigned long)diag.right_compare);
#if APP_ROBOT_INTEGRATED
    report_sensors();
#endif

    return b_waited && wait_and_poll(S2_HW_TEST_PAUSE_MS);
}

EXPORT void subsystem2_hw_test_task(INT stacd, void * p_exinf)
{
    const float pwm = S2_HW_TEST_PWM_PERCENT;
    bool        b_ok;

    (void)stacd;
    (void)p_exinf;
    drv_motor_stop();
    LOG_INFO(TAG_S2_HWTEST, "MOTOR BENCH TEST: raise wheels off the ground.");
    LOG_INFO(TAG_S2_HWTEST, "starts in %u ms; normal motion control disabled.",
             (UINT)S2_HW_TEST_START_MS);

    b_ok = (pwm > 0.0f && pwm <= 100.0f && S2_HW_TEST_RUN_MS > 0U &&
            S2_HW_TEST_RUN_MS <= 2000U);
    if (b_ok)
    {
        b_ok = wait_and_poll(S2_HW_TEST_START_MS);
    }
    if (b_ok)
    {
        b_ok = run_stage("LEFT ONLY (GP8 PWM, GP9 LOW)", pwm, 0.0f);
    }
    if (b_ok)
    {
        b_ok = run_stage("RIGHT ONLY (GP10 PWM, GP11 LOW)", 0.0f, pwm);
    }
    if (b_ok)
    {
        b_ok = run_stage("BOTH FORWARD", pwm, pwm);
    }
    if (b_ok)
    {
        b_ok = run_stage("LEFT PIVOT (right wheel only)", 0.0f, pwm);
    }
    if (b_ok)
    {
        b_ok = run_stage("RIGHT PIVOT (left wheel only)", pwm, 0.0f);
    }

    drv_motor_stop();
    LOG_INFO(TAG_S2_HWTEST, "%s; motors OFF. Inspect wheel behaviour manually.",
             b_ok ? "Sequence finished" : "Sequence aborted");

    /* Remain the sole motor owner; do not start normal control afterwards. */
    while (1)
    {
        tk_slp_tsk(TMO_FEVR);
    }
}

#endif

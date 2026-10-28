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
    line_barcode_msg_t line = {{0u, 0u, 0u}};
    IMU_DATA imu = {0};
    ER line_er = line_service_snapshot(&line);
    ER imu_er = subsystem4_get_latest(&imu);
    tm_printf((UB *)"[S2 HW] S3 IR heartbeat=%d bits=%u state=%u (paused)\n",
              line_er, (UINT)(line.data[0] & 7u), (UINT)(line.data[1] & 7u));
    tm_printf((UB *)"[S2 HW] S4 IMU read=%d sensor_ok=%d calibrated=%d samples=%lu errors=%lu\n",
              imu_er, imu.sensor_ok, imu.calibrated,
              (unsigned long)imu.sample_count, (unsigned long)imu.read_errors);
}
#endif

/* Poll while waiting, so encoder edges can be observed during each stage. */
static bool wait_and_poll(uint32_t duration_ms)
{
    SYSTIM start;
    SYSTIM now;

    if (tk_get_tim(&start) < E_OK) {
        return false;
    }
    do {
        drv_encoder_poll();
        if (tk_dly_tsk(S2_ENCODER_POLL_MS) < E_OK ||
            tk_get_tim(&now) < E_OK) {
            return false;
        }
    } while ((uint32_t)(now.lo - start.lo) < duration_ms);
    return true;
}

static bool run_stage(const char *name, float left, float right)
{
    int32_t left_ticks;
    int32_t right_ticks;
    bool waited;
    MOTOR_DIAGNOSTICS diag;
    ER check;

    drv_encoder_reset();
    /* Print before applying power; stop before printing the result. */
    tm_printf((UB *)"[S2 HW] %s: %u ms\n", name,
              (UINT)S2_HW_TEST_RUN_MS);
    drv_motor_set(left, right);
    check = drv_motor_diagnostics(&diag);
    waited = check == E_OK && wait_and_poll(S2_HW_TEST_RUN_MS);
    drv_motor_stop();
    drv_encoder_get_counts(&left_ticks, &right_ticks);
    tm_printf((UB *)"[S2 HW] STOP; ticks left=%ld right=%ld\n",
              (long)left_ticks, (long)right_ticks);
    tm_printf((UB *)"[S2 HW] PWM check=%d counting=%d/%d compare=%lu/%lu\n",
              check, diag.left_counting, diag.right_counting,
              (unsigned long)diag.left_compare, (unsigned long)diag.right_compare);
#if APP_ROBOT_INTEGRATED
    report_sensors();
#endif

    return waited && wait_and_poll(S2_HW_TEST_PAUSE_MS);
}

EXPORT void subsystem2_hw_test_task(INT stacd, void *exinf)
{
    const float pwm = S2_HW_TEST_PWM_PERCENT;
    bool ok;

    (void)stacd;
    (void)exinf;
    drv_motor_stop();
    tm_printf((UB *)"[S2 HW] MOTOR BENCH TEST: raise wheels off the ground.\n"
                       "[S2 HW] Starts in %u ms; normal motion control disabled.\n",
              (UINT)S2_HW_TEST_START_MS);

    ok = (pwm > 0.0f && pwm <= 100.0f &&
          S2_HW_TEST_RUN_MS > 0U && S2_HW_TEST_RUN_MS <= 2000U);
    if (ok) {
        ok = wait_and_poll(S2_HW_TEST_START_MS);
    }
    if (ok) {
        ok = run_stage("LEFT ONLY (GP8 PWM, GP9 LOW)", pwm, 0.0f);
    }
    if (ok) {
        ok = run_stage("RIGHT ONLY (GP10 PWM, GP11 LOW)", 0.0f, pwm);
    }
    if (ok) {
        ok = run_stage("BOTH FORWARD", pwm, pwm);
    }
    if (ok) {
        ok = run_stage("LEFT PIVOT (right wheel only)", 0.0f, pwm);
    }
    if (ok) {
        ok = run_stage("RIGHT PIVOT (left wheel only)", pwm, 0.0f);
    }

    drv_motor_stop();
    tm_printf((UB *)"[S2 HW] %s; motors OFF. Inspect wheel behaviour manually.\n",
              ok ? "Sequence finished" : "Sequence aborted");

    /* Remain the sole motor owner; do not start normal control afterwards. */
    while (1) {
        tk_slp_tsk(TMO_FEVR);
    }
}

#endif

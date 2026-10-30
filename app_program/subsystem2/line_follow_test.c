/* S2 motor driver + S3 sensor service, with one motor/queue-owning task.
 * TRACK layout: GP7=left, GP27=centre, GP0=right.
 */
#include "line_follow_test.h"

#if APP_LINE_FOLLOW_TEST
#include "drv_motor.h"
#include "subsystem2.h"
#include "line_follow_output.h"
#include "../subsystem3/line_service.h"

#define FOLLOW_BASE_PWM       17.8f
#define FOLLOW_INNER_PWM      S2_MIN_EFFECTIVE_PWM_PERCENT
/* Small open-loop right-wheel trim; encoder feedback is not used here. */
#define FOLLOW_RIGHT_BASE_PWM 20.2f
#define FOLLOW_START_MS       5000u
#define FOLLOW_DURATION_MS    30000u
#define FOLLOW_LINE_LOSS_MS   100u

static void follow_task(INT stacd, void *exinf)
{
    SYSTIM start, now;
    const char *reason = "startup failed";
    uint32_t last_report = 0u;
    uint32_t loss_since = 0u;
    bool line_missing = false;
    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"[LINE TEST] S2 motors + S3 TRACK sensors; 1=black\n"
              "[LINE TEST] GP7=left, GP27=centre, GP0=right\n"
              "[LINE TEST] starts in 5 seconds; runs for at most 30 seconds\n");
    tm_printf((UB *)"[LINE TEST] base=%d%% inner=%d%%; left=M1 GP8/9 right=M2 GP10/11\n",
              (int)FOLLOW_BASE_PWM, (int)FOLLOW_INNER_PWM);
    if (tk_dly_tsk(FOLLOW_START_MS) < E_OK) goto stopped;
    if (line_service_enable(true) < E_OK) goto stopped;
    /* Allow the two-sample debounce to settle before checking the line. */
    if (tk_dly_tsk(30u) < E_OK || tk_get_tim(&start) < E_OK) goto stopped;

    for (;;) {
        line_barcode_msg_t msg = {{0u, 0u, 0u}};
        ER er;
        if (tk_get_tim(&now) < E_OK) { reason = "clock failed"; break; }
        if ((uint32_t)(now.lo - start.lo) >= FOLLOW_DURATION_MS) {
            reason = "30-second test complete"; break;
        }
        /* Consume events so the S3 queue cannot fill. Barcode letters do
         * not issue turns in this line-following-only test. */
        for (unsigned i = 0u; i < 24u; ++i) {
            er = line_service_receive(&msg, TMO_POL);
            if (er == E_TMOUT) break;
            if (er < E_OK) { reason = "sensor queue failed"; goto stopped; }
            unsigned event = (msg.data[1] >> 3u) & 7u;
            if (event == LINE_BARCODE_EVT_FAULT) {
                reason = "sensor fault event"; goto stopped;
            }
        }
        er = line_service_snapshot(&msg);
        if (er < E_OK) { reason = "sensor snapshot stale/failed"; break; }
        unsigned state = msg.data[1] & 7u;
        if (state == LINE_BARCODE_FAULT || state == LINE_BARCODE_PAUSED) {
            reason = "sensor fault or paused"; break;
        }
        if (state == LINE_BARCODE_JUNCTION) {
            reason = "junction detected"; break;
        }
        float left, right;
        if (state == LINE_BARCODE_SEARCH ||
            !line_follow_output(msg.data[0], LINE_BARCODE_LAYOUT_TRACK,
                                FOLLOW_BASE_PWM, FOLLOW_INNER_PWM, &left, &right)) {
            drv_motor_stop();
            if (!line_missing) {
                line_missing = true;
                loss_since = now.lo;
                tm_printf((UB *)"[LINE TEST] line gap L=%u C=%u R=%u; PWM=0/0, waiting 100ms\n",
                      (UINT)((msg.data[0] >> 2u) & 1u),
                      (UINT)((msg.data[0] >> 1u) & 1u), (UINT)(msg.data[0] & 1u));
            }
            if ((uint32_t)(now.lo - loss_since) >= FOLLOW_LINE_LOSS_MS) {
                reason = "line absent or ambiguous for 100ms"; break;
            }
            if (tk_dly_tsk(LINE_BARCODE_PERIOD_MS) < E_OK) {
                reason = "control wait failed"; break;
            }
            continue;
        }
        if (line_missing) {
            if ((uint32_t)(now.lo - loss_since) >= FOLLOW_LINE_LOSS_MS) {
                reason = "line recovery exceeded 100ms"; break;
            }
            tm_printf((UB *)"[LINE TEST] line returned; resuming\n");
            line_missing = false;
        }
        /* Trim only the outer/straight wheel, retaining the inner-wheel
         * setting when steering right and zero output on line loss. */
        if (right == FOLLOW_BASE_PWM) right = FOLLOW_RIGHT_BASE_PWM;
        drv_motor_set(left, right);
        if ((uint32_t)(now.lo - last_report) >= 250u) {
            tm_printf((UB *)"[LINE TEST] L=%u C=%u R=%u PWM L=%d.%d R=%d.%d\n",
                      (UINT)((msg.data[0] >> 2u) & 1u),
                      (UINT)((msg.data[0] >> 1u) & 1u), (UINT)(msg.data[0] & 1u),
                      (int)left, ((int)(left * 10.0f)) % 10,
                      (int)right, ((int)(right * 10.0f)) % 10);
            last_report = now.lo;
        }
        if (tk_dly_tsk(LINE_BARCODE_PERIOD_MS) < E_OK) {
            reason = "control wait failed"; break;
        }
    }
stopped:
    drv_motor_stop();
    (void)line_service_enable(false);
    tm_printf((UB *)"[LINE TEST] STOPPED: %s; reboot to retry\n", reason);
    tk_slp_tsk(TMO_FEVR);
}

ER line_follow_test_start(void)
{
    ER er = drv_motor_init();
    MOTOR_DIAGNOSTICS diagnostics;
    if (er == E_OK) er = drv_motor_diagnostics(&diagnostics);
    if (er == E_OK) er = line_service_init();
    /* User confirmed all inputs read 1 on black and 0 on white. */
    if (er == E_OK) er = line_service_calibrate(7u, 0u);
    if (er == E_OK) er = line_service_set_layout(LINE_BARCODE_LAYOUT_TRACK);
    if (er == E_OK) er = line_service_start();
    if (er == E_OK) {
        ID tid = app_start_task(follow_task, 15, TP_PRC2, "line_follow_test");
        er = (tid < E_OK) ? tid : E_OK;
    }
    if (er < E_OK) drv_motor_stop();
    return er;
}
#endif

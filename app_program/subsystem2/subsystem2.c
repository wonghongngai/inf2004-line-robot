/*
 * subsystem2.c
 * Buddy 2 -- creates Motion Control kernel objects and starts its task.
 */
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

EXPORT ID mbf_motion_cmd;
EXPORT ID flg_motion_stop;
EXPORT ID mtx_motion_status;

/* The task owns writes to this value.  Other tasks can only copy it through
   subsystem2_get_status(), which holds mtx_motion_status. */
static MOTION_STATUS latest_status;

EXPORT ER subsystem2_init(void)
{
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};
    T_CFLG cflg = {0};
    ER er;

    /* Initialise hardware first; both drivers leave the vehicle stopped. */
    er = drv_motor_init();
    if (er < E_OK) {
        return er;
    }

    MOTOR_DIAGNOSTICS motor_diag;
    er = drv_motor_diagnostics(&motor_diag);
    tm_printf((UB *)"[MOTOR] PWM reset_done=%u counters L=%d R=%d check=%d\n",
              (UINT)(motor_diag.reset_done != 0u), motor_diag.left_counting,
              motor_diag.right_counting, er);
    if (er < E_OK) {
        drv_motor_stop();
        return er;
    }

    er = drv_encoder_init();
    if (er < E_OK) {
        drv_motor_stop();
        return er;
    }

    /* Buddy 2 receives copied MOTION_COMMAND structures here. */
    cmbf.mbfatr = TA_TFIFO;
    /* Kernel stores a length word alongside each copied message. */
    cmbf.bufsz = (INT)(S2_COMMAND_QUEUE_DEPTH * (sizeof(MOTION_COMMAND) + sizeof(UW)));
    cmbf.maxmsz = (INT)sizeof(MOTION_COMMAND);

    mbf_motion_cmd = tk_cre_mbf(&cmbf);

    if (mbf_motion_cmd < E_OK) {
        return mbf_motion_cmd;
    }

    cmtx.mtxatr = TA_INHERIT;

    mtx_motion_status = tk_cre_mtx(&cmtx);

    if (mtx_motion_status < E_OK) {
        return mtx_motion_status;
    }

    cflg.flgatr = TA_TFIFO;
    flg_motion_stop = tk_cre_flg(&cflg);
    if (flg_motion_stop < E_OK) return flg_motion_stop;

    latest_status.state = MOTION_STATE_INITIALISING;
    latest_status.active_command = MOTION_CMD_NONE;
    latest_status.fault = MOTION_FAULT_NONE;
    latest_status.motion_complete = false;

    latest_status.left_encoder_ticks = 0;
    latest_status.right_encoder_ticks = 0;

    latest_status.left_speed_cm_s = 0.0f;
    latest_status.right_speed_cm_s = 0.0f;
    latest_status.distance_cm = 0.0f;

    latest_status.left_pwm_percent = 0.0f;
    latest_status.right_pwm_percent = 0.0f;

#if APP_ROBOT_INTEGRATED
    return s2_legacy_example_init();
#else
    return E_OK;
#endif
}

EXPORT ER subsystem2_start(void)
{
    ID tid;

#if APP_ROBOT_INTEGRATED
    /* Start compatibility tasks first; motion starts only if all succeeded. */
    tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
                         "producer_task");
    if (tid < E_OK) return tid;
#endif

    tid = app_start_task(
#if S2_MOTOR_HW_TEST
        subsystem2_hw_test_task,
#else
        motion_task,
#endif
        S2_PRI_MOTION,
        TP_PRC2,
        "motion_task"
    );

    if (tid < E_OK) {
        return tid;
    }

    return E_OK;
}

EXPORT ER subsystem2_send_command(const MOTION_COMMAND *command)
{
    if (command == 0) {
        return E_PAR;
    }

#if S2_MOTOR_HW_TEST
    return E_OBJ; /* Bench pulses bypass the closed-loop command consumer. */
#else
    if (command->type == MOTION_CMD_STOP)
        return tk_set_flg(flg_motion_stop, 1u);
#if APP_ROBOT_INTEGRATED || APP_TEST || S2_MOTION_DEMO
    return E_OBJ; /* Autonomous controller/test owns commands in these modes. */
#endif
#endif

    /*
     * Non-blocking send.
     *
     * If the message buffer is full, the caller receives an error instead
     * of being blocked indefinitely.
     */
    return tk_snd_mbf(
        mbf_motion_cmd,
        command,
        (INT)sizeof(MOTION_COMMAND),
        TMO_POL
    );
}

EXPORT ER subsystem2_get_status(MOTION_STATUS *status)
{
    ER er;

    if (status == 0) {
        return E_PAR;
    }

    er = tk_loc_mtx(
        mtx_motion_status,
        TMO_FEVR
    );

    if (er < E_OK) {
        return er;
    }

    *status = latest_status;

    return tk_unl_mtx(mtx_motion_status);
}

/*
 * Only Motion Control should publish Buddy 2 status.
 *
 * Other subsystems should use subsystem2_get_status().
 */
void subsystem2_publish_status(const MOTION_STATUS *status)
{
    if (status == 0) {
        return;
    }

    if (tk_loc_mtx(mtx_motion_status, TMO_POL) == E_OK) {

        latest_status = *status;

        (void)tk_unl_mtx(mtx_motion_status);
    }
}

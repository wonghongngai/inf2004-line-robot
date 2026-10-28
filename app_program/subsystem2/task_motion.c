/*
 * task_motion.c
 * Buddy 2 -- single Motion Control RTOS task.
 *
 * Responsibilities:
 *
 *   - owns motor writes
 *   - owns encoder reads
 *   - receives Forward / Left / Right / Stop commands
 *   - runs encoder-based straight-line PID control
 *   - estimates speed and distance
 *   - detects encoder stalls
 *   - reports Motion Control status
 */
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "robot_control.h"

/*
 * Build the Motion Control configuration using the constants from
 * subsystem2.h.
 */
static MOTION_CONFIG build_config(void)
{
    MOTION_CONFIG cfg;

#if S2_CALIBRATION_READY

    /*
     * Real physical values after calibration.
     */
    cfg.left_ticks_per_cm =
        S2_LEFT_TICKS_PER_CM;

    cfg.right_ticks_per_cm =
        S2_RIGHT_TICKS_PER_CM;

    cfg.left_turn_ticks_per_degree =
        S2_LEFT_TURN_TICKS_PER_DEGREE;

    cfg.right_turn_ticks_per_degree =
        S2_RIGHT_TURN_TICKS_PER_DEGREE;

#else

    /*
     * Force validation failure until the physical robot has actually
     * been calibrated.
     *
     * This ensures the real motors do not move using guessed values.
     */
    cfg.left_ticks_per_cm =
        0.0f;

    cfg.right_ticks_per_cm =
        0.0f;

    cfg.left_turn_ticks_per_degree =
        0.0f;

    cfg.right_turn_ticks_per_degree =
        0.0f;

#endif

    cfg.base_pwm_percent =
        S2_BASE_PWM_PERCENT;

    cfg.minimum_effective_pwm_percent =
        S2_MIN_EFFECTIVE_PWM_PERCENT;

    cfg.maximum_pwm_percent =
        S2_MAX_PWM_PERCENT;

    cfg.distance_tolerance_cm =
        S2_DISTANCE_TOLERANCE_CM;

    cfg.angle_tolerance_deg =
        S2_ANGLE_TOLERANCE_DEG;

    cfg.encoder_stall_timeout_ms =
        S2_ENCODER_STALL_TIMEOUT_MS;

    cfg.kp =
        S2_PID_KP;

    cfg.ki =
        S2_PID_KI;

    cfg.kd =
        S2_PID_KD;

    cfg.integral_limit =
        S2_PID_INTEGRAL_LIMIT;

    cfg.correction_limit =
        S2_PID_CORRECTION_LIMIT;

    return cfg;
}

/*
 * Get current μT-Kernel system time in milliseconds.
 */
static uint32_t current_ms(void)
{
    SYSTIM now;

    (void)tk_get_tim(&now);

    return now.lo;
}

/*
 * Convert Motion Control's requested output into physical motor outputs.
 */
static void apply_outputs(
    const MOTION_STATUS *status
)
{
    /*
     * These states must never drive the motors.
     */
    if (
        (status->state == MOTION_STATE_FAULT) ||

        (status->state == MOTION_STATE_IDLE) ||

        (status->state == MOTION_STATE_STOPPING) ||

        (status->state == MOTION_STATE_INITIALISING)
    ) {

        drv_motor_stop();

    } else {

        drv_motor_set(
            status->left_pwm_percent,
            status->right_pwm_percent
        );
    }
}

/*
 * Process pending movement requests.
 *
 * Normal movement requests:
 *   newest request is kept.
 *
 * STOP:
 *   always immediately takes precedence.
 */
static bool receive_latest_command(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    MOTION_COMMAND command;

    MOTION_COMMAND latest_normal;

    bool have_normal = false;

    INT len;
    UINT pattern;
    if (tk_wai_flg(flg_motion_stop, 1u, TWF_ORW | TWF_CLR,
                   &pattern, TMO_POL) == E_OK) {
        command = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
        (void)motion_control_command(controller, &command, now_ms, left_ticks, right_ticks);
        for (unsigned i = 0; i < S2_COMMAND_QUEUE_DEPTH; ++i)
            if (tk_rcv_mbf(mbf_motion_cmd, &latest_normal, TMO_POL) <= 0) break;
        return true;
    }

    /*
     * Drain available messages so old normal movement requests cannot build
     * up and execute much later.
     */
    for (unsigned i = 0; i < S2_COMMAND_QUEUE_DEPTH &&
        (len =
            tk_rcv_mbf(
                mbf_motion_cmd,
                &command,
                TMO_POL
            )
        ) > 0; ++i
    ) {

        /*
         * Ignore malformed message sizes.
         */
        if (len !=
            (INT)sizeof(MOTION_COMMAND)) {

            continue;
        }

        /*
         * STOP immediately wins over pending normal commands.
         */
        if (command.type ==
            MOTION_CMD_STOP) {

            (void)motion_control_command(
                controller,
                &command,
                now_ms,
                left_ticks,
                right_ticks
            );

            /* Discard the rest of this batch so STOP cannot be followed by
               a stale movement request on the next poll. */
            for (unsigned j = 0; j < S2_COMMAND_QUEUE_DEPTH; ++j)
                if (tk_rcv_mbf(mbf_motion_cmd, &latest_normal, TMO_POL) <= 0) break;
            return true;
        }

        /*
         * Keep the newest normal command.
         */
        latest_normal =
            command;

        have_normal =
            true;
    }

    /*
     * Give Motion Control the latest normal command.
     */
    if (have_normal) {

        (void)motion_control_command(
            controller,
            &latest_normal,
            now_ms,
            left_ticks,
            right_ticks
        );
    }
    return false;
}

/*
 * Main Buddy 2 RTOS task.
 */
EXPORT void motion_task(
    INT stacd,
    void *exinf
)
{
    MOTION_CONTROLLER controller;

    MOTION_CONFIG cfg =
        build_config();

    MOTION_STATUS status;

    int32_t left_ticks;

    int32_t right_ticks;

    uint32_t next_control_ms;

    uint32_t now_ms;
    uint32_t last_report_ms = 0u;

    bool ready;
    bool manual_stop = false;

    /*
     * Task arguments are currently unused.
     */
    (void)stacd;

    (void)exinf;

    /*
     * Read starting encoder position.
     */
    drv_encoder_get_counts(
        &left_ticks,
        &right_ticks
    );

    now_ms =
        current_ms();

#if S2_CALIBRATION_READY

    ready =
        motion_control_init(
            &controller,
            &cfg,
            now_ms,
            left_ticks,
            right_ticks
        );

#else

    /*
     * Calibration is intentionally invalid.
     *
     * motion_control_init() therefore enters a configuration fault
     * instead of allowing physical movement.
     */
    ready =
        motion_control_init(
            &controller,
            &cfg,
            now_ms,
            left_ticks,
            right_ticks
        );

    tm_printf(
        (UB *)
        "motion: calibration not set; motors remain OFF\n"
    );

#endif

    if (!ready) {

        drv_motor_stop();
    }

    /*
     * Publish initial status.
     */
    motion_control_get_status(
        &controller,
        &status
    );

    subsystem2_publish_status(
        &status
    );

    next_control_ms =
        now_ms;

    while (1) {

        /*
         * TEMPORARY ENCODER FALLBACK:
         *
         * Poll encoder A/B states every 1 ms.
         *
         * Once the team owns one shared GPIO IRQ dispatcher, encoder edges
         * can be passed into the decoder from interrupts instead.
         */
        drv_encoder_poll();

        drv_encoder_get_counts(
            &left_ticks,
            &right_ticks
        );

        now_ms =
            current_ms();

        /*
         * Check for commands every 1 ms.
         *
         * This means STOP does not need to wait for the slower PID period.
         */
        if (receive_latest_command(
            &controller,
            now_ms,
            left_ticks,
            right_ticks
        )) manual_stop = true;

#if APP_ROBOT_INTEGRATED || S2_MOTION_DEMO
        if (manual_stop) {
            MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(&controller, &stop, now_ms, left_ticks, right_ticks);
        } else
#else
        (void)manual_stop;
#endif
        robot_control_step(&controller, now_ms, left_ticks, right_ticks);

        /*
         * Immediately apply safety-related command changes.
         *
         * Example:
         *
         *     FORWARD
         *        +
         *      STOP
         *        ↓
         *     PWM = 0
         *
         * without waiting for the next 10 ms PID execution.
         */
        motion_control_get_status(
            &controller,
            &status
        );

        apply_outputs(
            &status
        );

        subsystem2_publish_status(
            &status
        );

        /*
         * Run PID/state calculations at the Motion Control period.
         */
        if (
            (uint32_t)(
                now_ms -
                next_control_ms
            ) >=
            S2_CONTROL_PERIOD_MS
        ) {

            motion_control_step(
                &controller,
                now_ms,
                left_ticks,
                right_ticks
            );

            motion_control_get_status(
                &controller,
                &status
            );

            apply_outputs(
                &status
            );

            subsystem2_publish_status(
                &status
            );

            next_control_ms =
                now_ms;
        }

        if ((uint32_t)(now_ms - last_report_ms) >= 1000u) {
            tm_printf((UB *)"[MOTION] state=%d fault=%d ticks L=%ld R=%ld PWM=%d/%d\n",
                      status.state, status.fault, (long)left_ticks, (long)right_ticks,
                      (int)status.left_pwm_percent, (int)status.right_pwm_percent);
            last_report_ms = now_ms;
        }

        /*
         * Sleep until next encoder service interval.
         */
        tk_dly_tsk(
            S2_ENCODER_POLL_MS
        );
    }
}

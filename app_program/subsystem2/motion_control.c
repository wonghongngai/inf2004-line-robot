/*
 * motion_control.c
 * Buddy 2 -- Motion Control state machine, PID correction and fault handling.
 */
#include "motion_control.h"

/*
 * Return absolute value of a signed encoder count.
 */
static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*
 * Keep a float between minimum and maximum.
 */
static float clamp_float(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

/*
 * Verify that Motion Control has usable calibration values.
 *
 * This prevents the robot from moving using guessed / invalid values.
 */
static bool config_is_valid(const MOTION_CONFIG *cfg)
{
    if (cfg == 0) {
        return false;
    }

    return

        (cfg->left_ticks_per_cm > 0.0f) &&

        (cfg->right_ticks_per_cm > 0.0f) &&

        (cfg->left_turn_ticks_per_degree > 0.0f) &&

        (cfg->right_turn_ticks_per_degree > 0.0f) &&

        (cfg->base_pwm_percent > 0.0f) &&

        (cfg->minimum_effective_pwm_percent >= 0.0f) &&

        (cfg->maximum_pwm_percent > 0.0f) &&

        (cfg->minimum_effective_pwm_percent <=
         cfg->base_pwm_percent) &&

        (cfg->base_pwm_percent <=
         cfg->maximum_pwm_percent) &&

        (cfg->maximum_pwm_percent <= 100.0f) &&

        (cfg->distance_tolerance_cm >= 0.0f) &&

        (cfg->angle_tolerance_deg >= 0.0f) &&

        (cfg->encoder_stall_timeout_ms > 0U) &&

        (cfg->integral_limit >= 0.0f) &&

        (cfg->correction_limit >= 0.0f);
}

/*
 * Set both requested motor outputs to zero.
 */
static void stop_outputs(MOTION_CONTROLLER *controller)
{
    controller->status.left_pwm_percent = 0.0f;
    controller->status.right_pwm_percent = 0.0f;
}

/*
 * Clear the straight-line PID history.
 */
static void reset_pid(MOTION_CONTROLLER *controller)
{
    controller->pid_integral = 0.0f;

    controller->pid_previous_error = 0.0f;

    controller->pid_has_previous_error = false;
}

/*
 * Calculate straight-line PID correction.
 */
static float pid_update(
    MOTION_CONTROLLER *controller,
    float error,
    float dt_s
)
{
    float derivative = 0.0f;

    float output;

    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    /*
     * Integral term.
     */
    controller->pid_integral += error * dt_s;

    controller->pid_integral =
        clamp_float(
            controller->pid_integral,
            -controller->cfg.integral_limit,
            controller->cfg.integral_limit
        );

    /*
     * Derivative is only available after there is a previous error.
     */
    if (controller->pid_has_previous_error) {

        derivative =
            (error - controller->pid_previous_error) /
            dt_s;
    }

    controller->pid_previous_error = error;

    controller->pid_has_previous_error = true;

    output =
        (controller->cfg.kp * error) +
        (controller->cfg.ki * controller->pid_integral) +
        (controller->cfg.kd * derivative);

    /*
     * Limit how strongly PID may modify the base PWM.
     */
    return clamp_float(
        output,
        -controller->cfg.correction_limit,
        controller->cfg.correction_limit
    );
}

/*
 * Clamp a requested motor percentage.
 *
 * If movement is required, values below minimum_effective_pwm_percent
 * are raised to the minimum value which should physically move the wheel.
 */
static float moving_pwm(
    const MOTION_CONTROLLER *controller,
    float requested
)
{
    if (requested <= 0.0f) {
        return 0.0f;
    }

    return clamp_float(
        requested,
        controller->cfg.minimum_effective_pwm_percent,
        controller->cfg.maximum_pwm_percent
    );
}

/*
 * Convert encoder ticks into centimetres.
 */
static float ticks_to_cm(
    int32_t delta_ticks,
    float ticks_per_cm
)
{
    return
        (float)abs_i32(delta_ticks) /
        ticks_per_cm;
}

/*
 * Enter the fail-safe FAULT state.
 */
static void enter_fault(
    MOTION_CONTROLLER *controller,
    MOTION_FAULT fault
)
{
    stop_outputs(controller);

    controller->status.fault = fault;

    controller->status.state = MOTION_STATE_FAULT;

    controller->status.motion_complete = false;

    controller->status.active_command = MOTION_CMD_NONE;
}

/*
 * Common initialisation for Forward / Left / Right commands.
 */
static void begin_motion(
    MOTION_CONTROLLER *controller,
    MOTION_COMMAND_TYPE type,
    float target_value,
    MOTION_STATE state,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    controller->target_value = target_value;

    /*
     * Save encoder counts at the beginning of this command.
     */
    controller->command_start_left_ticks = left_ticks;

    controller->command_start_right_ticks = right_ticks;

    controller->previous_left_ticks = left_ticks;

    controller->previous_right_ticks = right_ticks;

    controller->previous_step_ms = now_ms;

    controller->last_left_progress_ms = now_ms;

    controller->last_right_progress_ms = now_ms;

    controller->status.active_command = type;

    controller->status.state = state;

    controller->status.motion_complete = false;

    controller->status.fault = MOTION_FAULT_NONE;

    controller->status.distance_cm = 0.0f;

    reset_pid(controller);
}

bool motion_control_init(
    MOTION_CONTROLLER *controller,
    const MOTION_CONFIG *config,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    if ((controller == 0) ||
        (config == 0)) {

        return false;
    }

    controller->cfg = *config;

    controller->configuration_valid =
        config_is_valid(config);

    controller->target_value = 0.0f;

    controller->command_start_left_ticks =
        left_ticks;

    controller->command_start_right_ticks =
        right_ticks;

    controller->previous_left_ticks =
        left_ticks;

    controller->previous_right_ticks =
        right_ticks;

    controller->previous_step_ms =
        now_ms;

    controller->last_left_progress_ms =
        now_ms;

    controller->last_right_progress_ms =
        now_ms;

    reset_pid(controller);

    controller->status.state =
        MOTION_STATE_INITIALISING;

    controller->status.active_command =
        MOTION_CMD_NONE;

    controller->status.fault =
        MOTION_FAULT_NONE;

    controller->status.motion_complete =
        false;

    controller->status.left_encoder_ticks =
        left_ticks;

    controller->status.right_encoder_ticks =
        right_ticks;

    controller->status.left_speed_cm_s =
        0.0f;

    controller->status.right_speed_cm_s =
        0.0f;

    controller->status.distance_cm =
        0.0f;

    stop_outputs(controller);

    /*
     * Invalid calibration is a fault.
     *
     * The motors remain stopped.
     */
    if (!controller->configuration_valid) {

        enter_fault(
            controller,
            MOTION_FAULT_CONFIGURATION
        );

        return false;
    }

    controller->status.state =
        MOTION_STATE_IDLE;

    return true;
}

MOTION_RESULT motion_control_command(
    MOTION_CONTROLLER *controller,
    const MOTION_COMMAND *command,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    if ((controller == 0) ||
        (command == 0)) {

        return MOTION_RESULT_INVALID_COMMAND;
    }

    /*
     * STOP is always accepted.
     *
     * However, STOP does NOT clear an existing FAULT.
     */
    if (command->type == MOTION_CMD_STOP) {

        stop_outputs(controller);

        controller->status.active_command =
            MOTION_CMD_NONE;

        if (controller->status.state !=
            MOTION_STATE_FAULT) {

            controller->status.state =
                MOTION_STATE_STOPPING;

            controller->status.motion_complete =
                true;
        }

        return MOTION_RESULT_OK;
    }

    /*
     * Explicit fault-reset command.
     */
    if (command->type ==
        MOTION_CMD_RESET_FAULT) {

        /*
         * A configuration fault cannot be cleared while the
         * underlying calibration is still invalid.
         */
        if ((!controller->configuration_valid) ||
            (controller->status.state !=
             MOTION_STATE_FAULT)) {

            return MOTION_RESULT_FAULT_LATCHED;
        }

        controller->status.fault =
            MOTION_FAULT_NONE;

        controller->status.state =
            MOTION_STATE_IDLE;

        controller->status.active_command =
            MOTION_CMD_NONE;

        controller->status.motion_complete =
            false;

        controller->previous_left_ticks =
            left_ticks;

        controller->previous_right_ticks =
            right_ticks;

        controller->previous_step_ms =
            now_ms;

        controller->last_left_progress_ms =
            now_ms;

        controller->last_right_progress_ms =
            now_ms;

        reset_pid(controller);

        return MOTION_RESULT_OK;
    }

    /*
     * Normal motion commands cannot run while a fault is latched.
     */
    if (controller->status.state ==
        MOTION_STATE_FAULT) {

        return MOTION_RESULT_FAULT_LATCHED;
    }

    /*
     * Prevent two movement commands from controlling the motors
     * simultaneously.
     */
    if (command->type == MOTION_CMD_FOLLOW_LINE &&
        command->value >= -1.0f && command->value <= 1.0f &&
        controller->status.active_command == MOTION_CMD_FOLLOW_LINE) {
        controller->target_value = command->value;
        return MOTION_RESULT_OK;
    }

    if (controller->status.state !=
        MOTION_STATE_IDLE) {

        return MOTION_RESULT_BUSY;
    }

    /*
     * Forward distance and turn angle must be positive.
     */
    if (command->type == MOTION_CMD_FOLLOW_LINE) {
        if (!(command->value >= -1.0f && command->value <= 1.0f)) {
            return MOTION_RESULT_INVALID_COMMAND;
        }
        begin_motion(controller, command->type, command->value,
                     MOTION_STATE_MOVING_FORWARD, now_ms, left_ticks, right_ticks);
        return MOTION_RESULT_OK;
    }

    if (!(command->value > 0.0f && command->value <= 100000.0f)) {

        return MOTION_RESULT_INVALID_COMMAND;
    }

    switch (command->type) {

    case MOTION_CMD_FORWARD_CM:

        begin_motion(
            controller,
            command->type,
            command->value,
            MOTION_STATE_MOVING_FORWARD,
            now_ms,
            left_ticks,
            right_ticks
        );

        return MOTION_RESULT_OK;

    case MOTION_CMD_TURN_LEFT_DEG:

        begin_motion(
            controller,
            command->type,
            command->value,
            MOTION_STATE_TURNING_LEFT,
            now_ms,
            left_ticks,
            right_ticks
        );

        return MOTION_RESULT_OK;

    case MOTION_CMD_TURN_RIGHT_DEG:

        begin_motion(
            controller,
            command->type,
            command->value,
            MOTION_STATE_TURNING_RIGHT,
            now_ms,
            left_ticks,
            right_ticks
        );

        return MOTION_RESULT_OK;

    default:

        return MOTION_RESULT_INVALID_COMMAND;
    }
}

/*
 * Return true once the encoder has not made progress for the configured
 * amount of time.
 */
static bool timed_out(
    uint32_t now_ms,
    uint32_t last_progress_ms,
    uint32_t timeout_ms
)
{
    return
        (uint32_t)(now_ms - last_progress_ms) >=
        timeout_ms;
}

/*
 * Update encoder count, speed and progress timestamps.
 */
static void update_measurements(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    uint32_t dt_ms;

    int32_t left_delta;

    int32_t right_delta;

    dt_ms =
        (uint32_t)(
            now_ms -
            controller->previous_step_ms
        );

    left_delta =
        left_ticks -
        controller->previous_left_ticks;

    right_delta =
        right_ticks -
        controller->previous_right_ticks;

    controller->status.left_encoder_ticks =
        left_ticks;

    controller->status.right_encoder_ticks =
        right_ticks;

    /*
     * Calculate wheel speed.
     */
    if (dt_ms > 0U) {

        float dt_s =
            (float)dt_ms / 1000.0f;

        controller->status.left_speed_cm_s =
            ticks_to_cm(
                left_delta,
                controller->cfg.left_ticks_per_cm
            ) / dt_s;

        controller->status.right_speed_cm_s =
            ticks_to_cm(
                right_delta,
                controller->cfg.right_ticks_per_cm
            ) / dt_s;
    }

    /*
     * Record when each encoder last changed.
     */
    if (left_delta != 0) {

        controller->last_left_progress_ms =
            now_ms;
    }

    if (right_delta != 0) {

        controller->last_right_progress_ms =
            now_ms;
    }

    controller->previous_left_ticks =
        left_ticks;

    controller->previous_right_ticks =
        right_ticks;

    controller->previous_step_ms =
        now_ms;
}

/*
 * Finish a movement normally.
 */
static void complete_motion(
    MOTION_CONTROLLER *controller
)
{
    stop_outputs(controller);

    controller->status.active_command =
        MOTION_CMD_NONE;

    controller->status.state =
        MOTION_STATE_IDLE;

    controller->status.motion_complete =
        true;
}

/*
 * Forward-motion logic.
 *
 * Both wheel distances are calculated from the encoder feedback.
 *
 * PID compares left vs right wheel travel:
 *
 *      error = left distance - right distance
 *
 * If the left wheel moves too far, PID reduces left PWM and increases
 * right PWM.
 */
static void run_forward(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks,
    float dt_s
)
{
    float left_cm;

    float right_cm;

    float average_cm;

    float remaining;

    float error;

    float correction;

    left_cm =
        ticks_to_cm(
            left_ticks -
            controller->command_start_left_ticks,
            controller->cfg.left_ticks_per_cm
        );

    right_cm =
        ticks_to_cm(
            right_ticks -
            controller->command_start_right_ticks,
            controller->cfg.right_ticks_per_cm
        );

    average_cm =
        (left_cm + right_cm) *
        0.5f;

    remaining =
        controller->target_value -
        average_cm;

    controller->status.distance_cm =
        average_cm;

    /*
     * Target reached.
     */
    if (controller->status.active_command != MOTION_CMD_FOLLOW_LINE && remaining <=
        controller->cfg.distance_tolerance_cm) {

        complete_motion(controller);

        return;
    }

    /*
     * Check left encoder progress.
     */
    if (timed_out(
            now_ms,
            controller->last_left_progress_ms,
            controller->cfg.encoder_stall_timeout_ms
        )) {

        enter_fault(
            controller,
            MOTION_FAULT_LEFT_ENCODER_STALL
        );

        return;
    }

    /*
     * Check right encoder progress.
     */
    if (timed_out(
            now_ms,
            controller->last_right_progress_ms,
            controller->cfg.encoder_stall_timeout_ms
        )) {

        enter_fault(
            controller,
            MOTION_FAULT_RIGHT_ENCODER_STALL
        );

        return;
    }

    /*
     * Positive error means the left wheel has travelled farther.
     */
    error =
        left_cm -
        right_cm;

    correction =
        pid_update(
            controller,
            error,
            dt_s
        );

    if (controller->status.active_command == MOTION_CMD_FOLLOW_LINE) {
        /* Steering right speeds up the left wheel. PID remains responsible
           for straight travel; reset its distance history during curves. */
        if (controller->target_value != 0.0f) {
            correction = -controller->target_value * controller->cfg.correction_limit;
            reset_pid(controller);
            controller->command_start_left_ticks = left_ticks;
            controller->command_start_right_ticks = right_ticks;
        }
    }

    /*
     * Slow the wheel that is ahead and speed up the wheel that is behind.
     */
    controller->status.left_pwm_percent =
        moving_pwm(
            controller,
            controller->cfg.base_pwm_percent -
            correction
        );

    controller->status.right_pwm_percent =
        moving_pwm(
            controller,
            controller->cfg.base_pwm_percent +
            correction
        );
}

/*
 * Left pivot.
 *
 * No reverse command is used.
 *
 * Left wheel remains stopped while the right wheel moves forward.
 */
static void run_left_turn(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t right_ticks
)
{
    float turned_deg;

    turned_deg =
        (float)abs_i32(
            right_ticks -
            controller->command_start_right_ticks
        ) /
        controller->cfg.right_turn_ticks_per_degree;

    /*
     * Target angle reached.
     */
    if ((controller->target_value -
         turned_deg) <=
        controller->cfg.angle_tolerance_deg) {

        complete_motion(controller);

        return;
    }

    /*
     * The active wheel should produce encoder movement.
     */
    if (timed_out(
            now_ms,
            controller->last_right_progress_ms,
            controller->cfg.encoder_stall_timeout_ms
        )) {

        enter_fault(
            controller,
            MOTION_FAULT_RIGHT_ENCODER_STALL
        );

        return;
    }

    controller->status.left_pwm_percent =
        0.0f;

    controller->status.right_pwm_percent =
        controller->cfg.base_pwm_percent;
}

/*
 * Right pivot.
 *
 * Right wheel remains stopped while left wheel moves forward.
 */
static void run_right_turn(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks
)
{
    float turned_deg;

    turned_deg =
        (float)abs_i32(
            left_ticks -
            controller->command_start_left_ticks
        ) /
        controller->cfg.left_turn_ticks_per_degree;

    /*
     * Target angle reached.
     */
    if ((controller->target_value -
         turned_deg) <=
        controller->cfg.angle_tolerance_deg) {

        complete_motion(controller);

        return;
    }

    /*
     * Check active left wheel.
     */
    if (timed_out(
            now_ms,
            controller->last_left_progress_ms,
            controller->cfg.encoder_stall_timeout_ms
        )) {

        enter_fault(
            controller,
            MOTION_FAULT_LEFT_ENCODER_STALL
        );

        return;
    }

    controller->status.left_pwm_percent =
        controller->cfg.base_pwm_percent;

    controller->status.right_pwm_percent =
        0.0f;
}

/*
 * One periodic Motion Control iteration.
 */
void motion_control_step(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
)
{
    uint32_t dt_ms;

    float dt_s;

    if (controller == 0) {
        return;
    }

    dt_ms =
        (uint32_t)(
            now_ms -
            controller->previous_step_ms
        );

    /*
     * First update encoder/speed measurements.
     */
    if (!controller->configuration_valid) {
        controller->status.left_encoder_ticks = left_ticks;
        controller->status.right_encoder_ticks = right_ticks;
        stop_outputs(controller);
        return;
    }

    update_measurements(
        controller,
        now_ms,
        left_ticks,
        right_ticks
    );

    dt_s =
        (dt_ms > 0U) ?
        ((float)dt_ms / 1000.0f) :
        0.0f;

    switch (controller->status.state) {

    case MOTION_STATE_MOVING_FORWARD:

        run_forward(
            controller,
            now_ms,
            left_ticks,
            right_ticks,
            dt_s
        );

        break;

    case MOTION_STATE_TURNING_LEFT:

        run_left_turn(
            controller,
            now_ms,
            right_ticks
        );

        break;

    case MOTION_STATE_TURNING_RIGHT:

        run_right_turn(
            controller,
            now_ms,
            left_ticks
        );

        break;

    case MOTION_STATE_STOPPING:

        stop_outputs(controller);

        controller->status.state =
            MOTION_STATE_IDLE;

        break;

    case MOTION_STATE_FAULT:

    case MOTION_STATE_IDLE:

    case MOTION_STATE_INITIALISING:

    default:

        stop_outputs(controller);

        break;
    }
}

/*
 * Return a copy of the current status.
 */
void motion_control_get_status(
    const MOTION_CONTROLLER *controller,
    MOTION_STATUS *status
)
{
    if ((controller != 0) &&
        (status != 0)) {

        *status =
            controller->status;
    }
}

/*
 * motion_control.h
 * Buddy 2 -- hardware-independent motion-control logic.
 *
 * This file intentionally uses ordinary C enums/structs/floats.  It does NOT
 * implement the Tribunal 1 minimum-bit data-representation change.
 */
#ifndef S2_MOTION_CONTROL_H
#define S2_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Commands Buddy 2 accepts.
 *
 * There is intentionally no backwards command.
 */
typedef enum {

    MOTION_CMD_NONE = 0,

    MOTION_CMD_FORWARD_CM,

    MOTION_CMD_TURN_LEFT_DEG,

    MOTION_CMD_TURN_RIGHT_DEG,

    MOTION_CMD_STOP,

    MOTION_CMD_RESET_FAULT,

    /* Continuous forward travel; value = steering -1 (left)..+1 (right). */
    MOTION_CMD_FOLLOW_LINE

} MOTION_COMMAND_TYPE;

/*
 * value means:
 *
 * FORWARD_CM      -> centimetres
 * TURN_LEFT_DEG   -> degrees
 * TURN_RIGHT_DEG  -> degrees
 * STOP            -> unused
 * RESET_FAULT     -> unused
 */
typedef struct {

    MOTION_COMMAND_TYPE type;

    float value;

} MOTION_COMMAND;

/*
 * Motion Control state machine.
 */
typedef enum {

    MOTION_STATE_INITIALISING = 0,

    MOTION_STATE_IDLE,

    MOTION_STATE_MOVING_FORWARD,

    MOTION_STATE_TURNING_LEFT,

    MOTION_STATE_TURNING_RIGHT,

    MOTION_STATE_STOPPING,

    MOTION_STATE_FAULT

} MOTION_STATE;

/*
 * Faults currently detectable by Buddy 2.
 */
typedef enum {

    MOTION_FAULT_NONE = 0,

    MOTION_FAULT_CONFIGURATION,

    MOTION_FAULT_LEFT_ENCODER_STALL,

    MOTION_FAULT_RIGHT_ENCODER_STALL

} MOTION_FAULT;

/*
 * Return result when a movement command is submitted.
 */
typedef enum {

    MOTION_RESULT_OK = 0,

    MOTION_RESULT_BUSY = -1,

    MOTION_RESULT_INVALID_COMMAND = -2,

    MOTION_RESULT_FAULT_LATCHED = -3

} MOTION_RESULT;

/*
 * Motion configuration.
 *
 * These values are deliberately separate from the controller state so that
 * the physical robot can be calibrated later.
 */
typedef struct {

    float left_ticks_per_cm;

    float right_ticks_per_cm;

    float left_turn_ticks_per_degree;

    float right_turn_ticks_per_degree;

    float base_pwm_percent;

    float minimum_effective_pwm_percent;

    float maximum_pwm_percent;

    float distance_tolerance_cm;

    float angle_tolerance_deg;

    uint32_t encoder_stall_timeout_ms;

    float kp;

    float ki;

    float kd;

    float integral_limit;

    float correction_limit;

} MOTION_CONFIG;

/*
 * Information exposed by Buddy 2 to the rest of the system.
 */
typedef struct {

    MOTION_STATE state;

    MOTION_COMMAND_TYPE active_command;

    MOTION_FAULT fault;

    bool motion_complete;

    int32_t left_encoder_ticks;

    int32_t right_encoder_ticks;

    float left_speed_cm_s;

    float right_speed_cm_s;

    float distance_cm;

    float left_pwm_percent;

    float right_pwm_percent;

} MOTION_STATUS;

/*
 * Internal Motion Control state.
 *
 * Other subsystems should not manipulate this directly.
 */
typedef struct {

    MOTION_CONFIG cfg;

    MOTION_STATUS status;

    int32_t command_start_left_ticks;

    int32_t command_start_right_ticks;

    int32_t previous_left_ticks;

    int32_t previous_right_ticks;

    uint32_t previous_step_ms;

    uint32_t last_left_progress_ms;

    uint32_t last_right_progress_ms;

    float target_value;

    float pid_integral;

    float pid_previous_error;

    bool pid_has_previous_error;

    bool configuration_valid;

} MOTION_CONTROLLER;

/*
 * Initialise Motion Control.
 *
 * Returns false if the calibration/configuration is invalid.
 */
bool motion_control_init(
    MOTION_CONTROLLER *controller,
    const MOTION_CONFIG *config,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
);

/*
 * Give Motion Control a new command.
 */
MOTION_RESULT motion_control_command(
    MOTION_CONTROLLER *controller,
    const MOTION_COMMAND *command,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
);

/*
 * One periodic Motion Control update.
 */
void motion_control_step(
    MOTION_CONTROLLER *controller,
    uint32_t now_ms,
    int32_t left_ticks,
    int32_t right_ticks
);

/*
 * Copy current Motion Control status.
 */
void motion_control_get_status(
    const MOTION_CONTROLLER *controller,
    MOTION_STATUS *status
);

#endif /* S2_MOTION_CONTROL_H */

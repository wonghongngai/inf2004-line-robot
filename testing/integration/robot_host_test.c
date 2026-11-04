#include "robot_host_stubs.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Inclusion makes adapter session state resettable between independent runs. */
#include "../../app_program/subsystem2/robot_control.c"

static IMU_DATA fake_imu;
static line_barcode_msg_t snapshot, queued;
static bool has_event, enabled;
static ER snapshot_error;
static unsigned checks;
#define EXPECT(x) do { assert(x); ++checks; } while (0)

int tm_printf(UB *format, ...) { (void)format; return 0; }
ER subsystem4_get_latest(IMU_DATA *out) { *out = fake_imu; return E_OK; }
ER line_service_calibrate(uint8_t b, uint8_t w) { return (b ^ w) == 7u ? E_OK : E_PAR; }
ER line_service_set_layout(uint8_t layout) { (void)layout; return E_OK; }
ER line_service_set_mapping(char l, char r, char s, char u)
{ (void)l; (void)r; (void)s; (void)u; return E_OK; }
ER line_service_enable(bool on) { enabled = on; has_event = false; return E_OK; }
ER line_service_snapshot(line_barcode_msg_t *msg) { *msg = snapshot; return snapshot_error; }
ER line_service_receive(line_barcode_msg_t *msg, TMO timeout)
{
    (void)timeout;
    if (!has_event) return E_TMOUT;
    *msg = queued; has_event = false; return E_OK;
}

static MOTION_CONFIG config(void)
{
    return (MOTION_CONFIG){
        .left_ticks_per_cm=10, .right_ticks_per_cm=10,
        .left_turn_ticks_per_degree=2, .right_turn_ticks_per_degree=2,
        .base_pwm_percent=50, .minimum_effective_pwm_percent=10,
        .maximum_pwm_percent=100, .distance_tolerance_cm=0.5f,
        .angle_tolerance_deg=1, .encoder_stall_timeout_ms=500,
        .kp=2, .integral_limit=100, .correction_limit=30
    };
}

static void reset(MOTION_CONTROLLER *ctl)
{
    MOTION_CONFIG cfg = config();
    armed = halted = turning = configured = false;
    last_imu_sample = imu_count = imu_errors = resumed_at = turn_started = 0;
    has_event = enabled = false; snapshot_error = E_OK;
    memset(&fake_imu, 0, sizeof(fake_imu));
    fake_imu.sensor_ok = fake_imu.calibrated = TRUE;
    fake_imu.sample_count = 1;
    snapshot = (line_barcode_msg_t){{3u | (2u << 3u), LINE_BARCODE_FOLLOW, 0u}};
    EXPECT(motion_control_init(ctl, &cfg, 0, 0, 0));
}

static void follow(MOTION_CONTROLLER *ctl)
{
    reset(ctl);
    robot_control_step(ctl, 5000, 0, 0);
    EXPECT(armed && enabled);
    EXPECT(ctl->status.state == MOTION_STATE_IDLE);
    robot_control_step(ctl, 5030, 0, 0);
    motion_control_step(ctl, 5040, 1, 1);
    EXPECT(ctl->status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT(ctl->status.left_pwm_percent > 0);
}

static void stopped(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(halted && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
    fake_imu.motion_state = IMU_MOTION_NORMAL;
    fake_imu.sensor_ok = TRUE;
    fake_imu.sample_count++;
    robot_control_step(ctl, now + 1, 1, 1);
    EXPECT(halted && ctl->status.left_pwm_percent == 0);
}

int main(void)
{
    MOTION_CONTROLLER ctl;
    reset(&ctl);
    fake_imu.calibrated = FALSE;
    robot_control_step(&ctl, 6000, 0, 0);
    EXPECT(!armed && ctl.status.left_pwm_percent == 0);

    follow(&ctl);
    snapshot.data[0] = 3u | (0u << 3u);
    robot_control_step(&ctl, 5041, 1, 1);
    motion_control_step(&ctl, 5050, 2, 2);
    EXPECT(ctl.status.left_pwm_percent < ctl.status.right_pwm_percent);
    snapshot.data[0] = 3u | (4u << 3u);
    robot_control_step(&ctl, 5051, 2, 2);
    motion_control_step(&ctl, 5060, 3, 3);
    EXPECT(ctl.status.left_pwm_percent > ctl.status.right_pwm_percent);

    for (unsigned action = 0; action < 4; ++action) {
        follow(&ctl);
        queued = (line_barcode_msg_t){{(uint8_t)(action << 6u),
                    LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
        has_event = true;
        robot_control_step(&ctl, 5041, 1, 1);
        if (action == LINE_BARCODE_CMD_STRAIGHT) {
            EXPECT(!turning && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
            continue;
        }
        EXPECT(turning && !enabled);
        EXPECT(ctl.target_value == (action == LINE_BARCODE_CMD_UTURN ? 180 : 90));
        bool right = action == LINE_BARCODE_CMD_RIGHT;
        EXPECT(ctl.status.active_command == (right ? MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG));
        int ticks = (int)(ctl.target_value * 2) + 1;
        motion_control_step(&ctl, 5050, right ? ticks : 1, right ? 1 : ticks);
        robot_control_step(&ctl, 5051, right ? ticks : 1, right ? 1 : ticks);
        EXPECT(!turning && enabled);
    }
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_COLLISION; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.event = IMU_EVENT_TILT; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.sensor_ok = FALSE; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.read_errors++; stopped(&ctl, 5041);
    follow(&ctl); stopped(&ctl, 5101); /* stale IMU heartbeat */
    follow(&ctl); snapshot_error = E_TMOUT; stopped(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_FAULT; stopped(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_SEARCH; stopped(&ctl, 5041);
    for (unsigned event = 0; event < 3; ++event) {
        const unsigned events[] = {LINE_BARCODE_EVT_LOST, LINE_BARCODE_EVT_FAULT, LINE_BARCODE_EVT_JUNCTION};
        follow(&ctl); queued = snapshot;
        queued.data[1] = (uint8_t)(events[event] << 3u); has_event = true;
        stopped(&ctl, 5041);
    }
    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    snapshot.data[1] = LINE_BARCODE_FAULT;
    stopped(&ctl, 5041);
    EXPECT(!turning); /* Newer fault takes precedence over queued turn. */

    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(turning);
    fake_imu.sample_count++;
    stopped(&ctl, 15042); /* Turn deadline with a fresh IMU sample. */

    follow(&ctl);
    fake_imu.motion_state = IMU_MOTION_HUMP;
    fake_imu.event = IMU_EVENT_HUMP;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!halted && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    follow(&ctl);
    motion_control_step(&ctl, 5600, 1, 1);
    EXPECT(ctl.status.state == MOTION_STATE_FAULT);
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);

    /* The original standalone distance and turn interface remains usable. */
    reset(&ctl);
    MOTION_COMMAND cmd = {MOTION_CMD_FORWARD_CM, 20};
    EXPECT(motion_control_command(&ctl, &cmd, 0, 0, 0) == MOTION_RESULT_OK);
    motion_control_step(&ctl, 10, 200, 200);
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90};
    EXPECT(motion_control_command(&ctl, &cmd, 20, 200, 200) == MOTION_RESULT_OK);
    motion_control_step(&ctl, 30, 380, 200);
    EXPECT(ctl.status.state == MOTION_STATE_IDLE);
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM, NAN};
    EXPECT(motion_control_command(&ctl, &cmd, 40, 380, 200) == MOTION_RESULT_INVALID_COMMAND);
    printf("robot host tests: %u checks passed\n", checks);
    return 0;
}

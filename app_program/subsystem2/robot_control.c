/* Adapter around the existing S3/S4 public interfaces. */
#include "subsystem2.h"
#include "robot_control.h"

#if APP_ROBOT_INTEGRATED && !APP_TEST
#include "../subsystem3/line_service.h"
#include "../subsystem4/subsystem4.h"

static bool armed, halted, line_halt, turning, configured;
static uint32_t last_imu_sample, imu_count, imu_errors, resumed_at, turn_started;
static uint32_t rearm_since;
static int32_t rearm_l, rearm_r;

/* Line stops (lost, unmapped junction, bad snapshot) may re-arm by
   themselves; any other stop is a safety stop and latches until reboot. */
static void stop_robot(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                       int32_t r, const char *reason, bool recoverable)
{
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(ctl, &stop, now, l, r);
    if (!halted) {
        (void)line_service_enable(false);
        tm_printf((UB *)"[ROBOT] stopped: %s; %s\n", reason,
                  recoverable ? "re-arms when back on the line"
                              : "reboot to re-arm");
        line_halt = recoverable;
        rearm_since = now;
        rearm_l = l;
        rearm_r = r;
    } else if (!recoverable) {
        line_halt = false;
    }
    halted = true;
}

static void halt(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r,
                 const char *reason)
{
    stop_robot(ctl, now, l, r, reason, false);
}

static void halt_line(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                      int32_t r, const char *reason)
{
    stop_robot(ctl, now, l, r, reason, true);
}

/* Paused S3 snapshots still carry calibrated sensor bits. Re-arm only after
   centre and right have both seen black, with the wheels still, for
   S2_LINE_REARM_MS; moving the robot by hand restarts the wait. */
static bool line_rearm_ready(uint32_t now, int32_t l, int32_t r)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    bool ready = line_service_snapshot(&msg) >= E_OK &&
                 (msg.data[0] & 3u) == 3u && l == rearm_l && r == rearm_r;

    if (!ready) {
        rearm_since = now;
        rearm_l = l;
        rearm_r = r;
        return false;
    }
    return (uint32_t)(now - rearm_since) >= S2_LINE_REARM_MS;
}

static bool line_unsafe_active;
static uint32_t line_unsafe_since;

/* A FAULT stops at once. Stale, paused, searching or absent-position
   snapshots must persist for S2_LINE_UNSAFE_MS before stopping, so one late
   or debouncing sample cannot latch a stop. S3 queues LOST after 100 ms. */
static bool line_snapshot_ok(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                             int32_t r, ER er, const line_barcode_msg_t *msg,
                             bool need_position)
{
    unsigned state = msg->data[1] & 7u;
    unsigned pos = (msg->data[0] >> 3u) & 7u;
    bool fault = er >= E_OK && state == LINE_BARCODE_FAULT;

    if (er >= E_OK && !fault && state != LINE_BARCODE_SEARCH &&
        state != LINE_BARCODE_PAUSED && (!need_position || pos < 6u)) {
        line_unsafe_active = false;
        return true;
    }
    if (!line_unsafe_active) {
        line_unsafe_active = true;
        line_unsafe_since = now;
    }
    if (fault || (uint32_t)(now - line_unsafe_since) >= S2_LINE_UNSAFE_MS) {
        tm_printf((UB *)"[ROBOT] line er=%d state=%u pos=%u sensors=%u\n",
                  (int)er, state, pos, (unsigned)(msg->data[0] & 7u));
        halt_line(ctl, now, l, r, "line snapshot unsafe or stale");
    }
    return false;
}

void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    IMU_DATA imu;
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    MOTION_COMMAND cmd;
    ER er;

    if (halted) {
        if (!line_halt || !line_rearm_ready(now, l, r)) {
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(ctl, &cmd, now, l, r);
            return;
        }
        halted = false;
        line_halt = false;
        turning = false;
        er = line_service_enable(true);
        if (er < E_OK) {
            halt(ctl, now, l, r, "line resume failed");
            return;
        }
        resumed_at = now;
        line_unsafe_active = false;
        tm_printf((UB *)"[ROBOT] line found, re-armed\n");
        return;
    }
    if (ctl->status.state == MOTION_STATE_FAULT) {
        halt(ctl, now, l, r, "motion fault / calibration missing");
        return;
    }
    er = subsystem4_get_latest(&imu);
    if (er < E_OK) {
        halt(ctl, now, l, r, "IMU snapshot failed");
        return;
    }
    if (imu.sample_count != imu_count) {
        imu_count = imu.sample_count;
        last_imu_sample = now;
    }
    if (!armed) {
        /* Remain stationary throughout IMU baseline calibration. */
        if (!S2_LINE_CALIBRATION_READY || now < S2_ROBOT_START_MS ||
            !imu.sensor_ok || !imu.calibrated || imu_count == 0 ||
            (uint32_t)(now - last_imu_sample) > S2_IMU_STALE_MS) return;
        if (!configured) {
            er = line_service_calibrate(S2_LINE_BLACK_RAW, S2_LINE_WHITE_RAW);
            if (er == E_OK) er = line_service_set_layout(S2_LINE_LAYOUT);
            if (er == E_OK) er = line_service_set_mapping('A', 'B', 'C', 'Z');
            if (er == E_OK) er = line_service_enable(true);
            if (er < E_OK) {
                halt(ctl, now, l, r, "line configuration failed");
                return;
            }
            configured = true;
            resumed_at = now;
            imu_errors = imu.read_errors;
            armed = true;
            tm_printf((UB *)"[ROBOT] armed: A=left B=right C=straight Z=U-turn\n");
        }
    }
    if (!imu.sensor_ok || !imu.calibrated ||
        imu.read_errors != imu_errors ||
        (uint32_t)(now - last_imu_sample) > S2_IMU_STALE_MS ||
        imu.motion_state == IMU_MOTION_TILT ||
        imu.motion_state == IMU_MOTION_COLLISION ||
        imu.motion_state == IMU_MOTION_FAULT ||
        imu.event == IMU_EVENT_TILT || imu.event == IMU_EVENT_COLLISION ||
        imu.event == IMU_EVENT_FAULT) {
        halt(ctl, now, l, r, "IMU unsafe or stale");
        return;
    }
    /* Hump is terrain information; keep following, without inventing speed
       compensation before physical tuning. */
    if (turning) {
        if ((uint32_t)(now - turn_started) > S2_ROBOT_TURN_TIMEOUT_MS) {
            halt(ctl, now, l, r, "turn timeout");
        } else if (ctl->status.state == MOTION_STATE_IDLE && ctl->status.motion_complete) {
            turning = false;
            er = line_service_enable(true);
            resumed_at = now;
            line_unsafe_active = false;
            if (er < E_OK) halt(ctl, now, l, r, "line resume failed");
        }
        return;
    }
    /* Permit two sensor periods after enable/reset, with zero motor output. */
    if ((uint32_t)(now - resumed_at) < 30u) return;

    /* A newer snapshot fault must win over an older queued turn command. */
    er = line_service_snapshot(&msg);
    if (!line_snapshot_ok(ctl, now, l, r, er, &msg, false)) return;

    /* Drain events with a fixed budget so control cannot starve. */
    for (unsigned i = 0; i < 24u; ++i) {
        er = line_service_receive(&msg, TMO_POL);
        if (er == E_TMOUT) break;
        if (er < E_OK) {
            halt(ctl, now, l, r, "line receive failed");
            return;
        }
        uint8_t event = (msg.data[1] >> 3u) & 7u;
        if (event == LINE_BARCODE_EVT_LOST || event == LINE_BARCODE_EVT_FAULT ||
            event == LINE_BARCODE_EVT_JUNCTION) {
            halt_line(ctl, now, l, r, "line lost, fault, or unmapped junction");
            return;
        }
        if (event == LINE_BARCODE_EVT_COMMAND) {
            unsigned action = msg.data[0] >> 6u;
            if (action == LINE_BARCODE_CMD_STRAIGHT) continue;
            er = line_service_enable(false);
            if (er < E_OK) {
                halt(ctl, now, l, r, "line pause failed");
                return;
            }
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(ctl, &cmd, now, l, r);
            motion_control_step(ctl, now, l, r); /* STOPPING -> IDLE */
            cmd.type = action == LINE_BARCODE_CMD_RIGHT ?
                       MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG;
            cmd.value = action == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
            if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK) {
                halt(ctl, now, l, r, "turn rejected");
                return;
            }
            turning = true;
            turn_started = now;
            return;
        }
    }
    er = line_service_snapshot(&msg);
    if (!line_snapshot_ok(ctl, now, l, r, er, &msg, true)) return;
    unsigned pos = (msg.data[0] >> 3u) & 7u;
    cmd.type = MOTION_CMD_FOLLOW_LINE;
    /* S3's positions increase from far-left to far-right. */
    cmd.value = pos <= 4u ? ((float)pos - 2.0f) * 0.5f : 0.0f;
    if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK)
        halt(ctl, now, l, r, "follow command rejected");
}

#elif S2_MOTION_DEMO && !APP_TEST

/* Standalone closed-loop sequence uses the same motor task and encoders. */
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    static const MOTION_COMMAND sequence[] = {
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_RIGHT_DEG, 90.0f}
    };
    static unsigned stage;
    static bool waiting, done;
    static uint32_t started, pause_until = S2_ROBOT_START_MS;
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    if (done) {
        (void)motion_control_command(ctl, &stop, now, l, r);
        return;
    }
    if (ctl->status.state == MOTION_STATE_FAULT) return;
    if (waiting) {
        if (ctl->status.state == MOTION_STATE_IDLE && ctl->status.motion_complete) {
            waiting = false;
            pause_until = now + 1500u;
            if (++stage == sizeof(sequence) / sizeof(sequence[0])) {
                done = true;
                tm_printf((UB *)"[S2 DEMO] finished; motors OFF\n");
            }
        } else if ((uint32_t)(now - started) > S2_DEMO_TIMEOUT_MS) {
            (void)motion_control_command(ctl, &stop, now, l, r);
            done = true;
            tm_printf((UB *)"[S2 DEMO] timeout; motors OFF\n");
        }
    } else if ((int32_t)(now - pause_until) >= 0) {
        if (motion_control_command(ctl, &sequence[stage], now, l, r) == MOTION_RESULT_OK) {
            waiting = true;
            started = now;
            tm_printf((UB *)"[S2 DEMO] stage %u started\n", stage + 1u);
        }
    }
}
#else
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    (void)ctl; (void)now; (void)l; (void)r;
}
#endif

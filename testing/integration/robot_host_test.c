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
static char applied_map[5];   /* last map handed to S3, as "LRSU" */
ER line_service_set_mapping(char l, char r, char s, char u)
{
    applied_map[0] = l; applied_map[1] = r; applied_map[2] = s; applied_map[3] = u;
    return E_OK;
}
/* S3 lookup against the last applied map: slot = LEFT, RIGHT, STRAIGHT, U. */
ER line_service_lookup(uint8_t letter, uint8_t *command)
{
    for (unsigned slot = 0u; slot < 4u; ++slot) {
        if (applied_map[slot] != '\0' && (uint8_t)(applied_map[slot] - 'A') == letter) {
            *command = (uint8_t)slot;
            return E_OK;
        }
    }
    return E_NOEXS;
}
static unsigned fake_black, fake_white;
void line_service_barcode_debug(unsigned *black, unsigned *white, uint8_t *letter)
{ *black = fake_black; *white = fake_white; *letter = 0xffu; }
static unsigned frame_calls;
void line_service_barcode_frame(line_service_barcode_frame_t *frame)
{ ++frame_calls; memset(frame, 0, sizeof(*frame)); }
ER line_service_enable(bool on) { enabled = on; has_event = false; return E_OK; }
ER line_service_snapshot(line_barcode_msg_t *msg) { *msg = snapshot; return snapshot_error; }
ER line_service_receive(line_barcode_msg_t *msg, TMO timeout)
{
    (void)timeout;
    if (!has_event) return E_TMOUT;
    *msg = queued; has_event = false; return E_OK;
}

/* S1 command queue (#48): a FIFO the test fills; the VC polls it. */
static unsigned char commands[16];
static unsigned command_head, command_count, command_polls;
INT subsystem1_receive_command(S1_COMMAND *command, TMO timeout)
{
    (void)timeout;
    ++command_polls;
    if (command_count == 0u) return E_TMOUT;
    command->bytes[0] = commands[command_head++ % 16u];
    --command_count;
    return 1;
}
static void send_command(unsigned code)
{
    commands[(command_head + command_count++) % 16u] = (unsigned char)code;
}

/* S1 letter-map queue and telemetry sink (#72). */
static S1_MAPPING pending_map;
static bool has_map;
INT subsystem1_receive_mapping(S1_MAPPING *mapping, TMO timeout)
{
    (void)timeout;
    if (!has_map) return E_TMOUT;
    *mapping = pending_map; has_map = false;
    return (INT)S1_MAPPING_BYTES;
}
static void send_map(const char *letters) /* "LRSU", e.g. "FAZW" */
{
    unsigned char codes[S1_MAP_SLOTS];
    for (unsigned i = 0u; i < S1_MAP_SLOTS; ++i) codes[i] = (unsigned char)(letters[i] - 'A');
    EXPECT(s1_mapping_set(&pending_map, codes));
    has_map = true;
}
static S1_TELEMETRY published;
static unsigned publish_count;
ER subsystem1_publish_telemetry(const S1_TELEMETRY *packet)
{
    published = *packet; ++publish_count; return E_OK;
}
static int field(S1_FIELD f)
{
    int value = -1;
    EXPECT(s1_telemetry_get(&published, f, &value));
    return value;
}

/* S5 executor: modelled as idle (no obstacle, no start-up hold), so these
 * tests exercise the VC's own line/barcode/IMU logic. Avoidance hand-over is
 * not covered here. */
static unsigned s5_steps;
void s5_exec_set_following(S5_EXEC *p_x, BOOL b_following)
{ (void)p_x; (void)b_following; }
BOOL s5_exec_startup_hold(void) { return FALSE; }
BOOL s5_exec_step(S5_EXEC *p_x, const S5_EXEC_OPS *p_ops, void *p_ctx, UW now_ms)
{ (void)p_x; (void)p_ops; (void)p_ctx; (void)now_ms; ++s5_steps; return FALSE; }

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

/* #8: line following with all_test's motor numbers, checked on the motion
 * controller alone. Expected values come from the S2_FOLLOW_* constants. */
static void follow_tuning_tests(void)
{
    MOTION_CONTROLLER m;
    MOTION_CONFIG cfg = config();
    MOTION_COMMAND follow = {MOTION_CMD_FOLLOW_LINE, 0.0f};
    const float L = S2_FOLLOW_LEFT_PWM_PERCENT, R = S2_FOLLOW_RIGHT_PWM_PERCENT;
    const float boost = S2_FOLLOW_BOOST_PWM_PERCENT;
    const float inner = S2_FOLLOW_INNER_PWM_PERCENT;
    const float hard = S2_FOLLOW_HARD_INNER_PWM_PERCENT;
    const float step = S2_FOLLOW_RAMP_PWM_PER_S * 0.010f;   /* 10 ms period */
    uint32_t now = 0u;

    cfg.minimum_effective_pwm_percent = 15.0f;  /* as integrated; follow ignores it */
    cfg.follow_left_pwm_percent = L;
    cfg.follow_right_pwm_percent = R;
    cfg.follow_boost_pwm_percent = boost;
    cfg.follow_inner_pwm_percent = inner;
    cfg.follow_hard_inner_pwm_percent = hard;
    cfg.follow_ramp_pwm_per_s = S2_FOLLOW_RAMP_PWM_PER_S;
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);

    /* From rest: one ramp step per 10 ms, from 0 (no moving floor), up to
     * the per-wheel cruise. The wheels never move: no stall fault either. */
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(fabsf(m.status.left_pwm_percent - step) < 1e-4f &&
           fabsf(m.status.right_pwm_percent - step) < 1e-4f);
    for (int i = 0; i < 200; ++i) {
        float before = m.status.right_pwm_percent;
        now += 10u;
        motion_control_step(&m, now, 0, 0);
        EXPECT(m.status.right_pwm_percent <= before + step + 1e-4f);
    }
    EXPECT(m.status.state != MOTION_STATE_FAULT);              /* 2 s, no ticks */
    EXPECT(fabsf(m.status.left_pwm_percent - L) < 1e-4f &&
           fabsf(m.status.right_pwm_percent - R) < 1e-4f);

    /* Soft right: inner (right) wheel drops at once to the inner PWM, the
     * outer ramps to cruise + boost. */
    follow.value = 0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(m.status.right_pwm_percent == inner);
    EXPECT(fabsf(m.status.left_pwm_percent - (L + step)) < 1e-4f);
    for (int i = 0; i < 100; ++i) { now += 10u; motion_control_step(&m, now, 0, 0); }
    EXPECT(fabsf(m.status.left_pwm_percent - (L + boost)) < 1e-4f);

    /* Hard right: inner wheel to the hard PWM (stopped). */
    follow.value = 1.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(m.status.right_pwm_percent == hard &&
           fabsf(m.status.left_pwm_percent - (L + boost)) < 1e-4f);

    /* Soft and hard left mirror it, on the other wheel. */
    follow.value = -0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(m.status.left_pwm_percent == inner);
    for (int i = 0; i < 100; ++i) { now += 10u; motion_control_step(&m, now, 0, 0); }
    EXPECT(fabsf(m.status.right_pwm_percent - (R + boost)) < 1e-4f);
    follow.value = -1.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(m.status.left_pwm_percent == hard);

    /* Back to straight: the outer wheel slows at once to cruise. */
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now += 10u;
    motion_control_step(&m, now, 0, 0);
    EXPECT(fabsf(m.status.right_pwm_percent - R) < 1e-4f);
    EXPECT(m.status.state != MOTION_STATE_FAULT);

    /* Distance moves keep base PWM and the stall fault. */
    MOTION_COMMAND fwd = {MOTION_CMD_FORWARD_CM, 50.0f};
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    EXPECT(motion_control_command(&m, &fwd, 0u, 0, 0) == MOTION_RESULT_OK);
    motion_control_step(&m, 10u, 5, 5);
    EXPECT(m.status.left_pwm_percent == cfg.base_pwm_percent);
    motion_control_step(&m, 10u + cfg.encoder_stall_timeout_ms + 10u, 5, 5);
    EXPECT(m.status.state == MOTION_STATE_FAULT);

    /* Invalid numbers are rejected: outer wheel above the maximum, negatives. */
    cfg.follow_boost_pwm_percent = 100.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg.follow_boost_pwm_percent = boost;
    cfg.follow_inner_pwm_percent = -1.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg.follow_inner_pwm_percent = inner;
    cfg.follow_ramp_pwm_per_s = -1.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
}

/* The integrated build's configuration: every number all_test drives with
 * (#8), as build_config() in task_motion.c sets it. */
static MOTION_CONFIG alltest_config(void)
{
    MOTION_CONFIG c = {0};
    c.left_ticks_per_cm = S2_ENC_LEFT_TICKS_PER_CM;
    c.right_ticks_per_cm = S2_ENC_RIGHT_TICKS_PER_CM;
    c.left_turn_ticks_per_degree = S2_LEFT_TURN_TICKS_PER_DEGREE;
    c.right_turn_ticks_per_degree = S2_RIGHT_TURN_TICKS_PER_DEGREE;
    c.base_pwm_percent = S2_BASE_PWM_PERCENT;
    c.minimum_effective_pwm_percent = S2_MIN_EFFECTIVE_PWM_PERCENT;
    c.maximum_pwm_percent = S2_MAX_PWM_PERCENT;
    c.distance_tolerance_cm = S2_DISTANCE_TOLERANCE_CM;
    c.angle_tolerance_deg = S2_ANGLE_TOLERANCE_DEG;
    c.encoder_stall_timeout_ms = S2_ENCODER_STALL_TIMEOUT_MS;
    c.integral_limit = S2_PID_INTEGRAL_LIMIT;
    c.correction_limit = S2_PID_CORRECTION_LIMIT;
    c.follow_left_pwm_percent = S2_FOLLOW_LEFT_PWM_PERCENT;
    c.follow_right_pwm_percent = S2_FOLLOW_RIGHT_PWM_PERCENT;
    c.follow_boost_pwm_percent = S2_FOLLOW_BOOST_PWM_PERCENT;
    c.follow_inner_pwm_percent = S2_FOLLOW_INNER_PWM_PERCENT;
    c.follow_hard_inner_pwm_percent = S2_FOLLOW_HARD_INNER_PWM_PERCENT;
    c.follow_ramp_pwm_per_s = S2_FOLLOW_RAMP_PWM_PER_S;
    c.follow_straight_kp = S2_FOLLOW_STRAIGHT_KP;
    c.follow_straight_max_trim = S2_FOLLOW_STRAIGHT_MAX_TRIM;
    c.follow_straight_trim_rate = S2_FOLLOW_STRAIGHT_TRIM_RATE;
    c.follow_straight_deadband_cm = S2_FOLLOW_STRAIGHT_DEADBAND_CM;
    c.follow_scan_scale = S2_FOLLOW_SCAN_SCALE;
    c.follow_scan_left_inner_pwm_percent = S2_FOLLOW_SCAN_LEFT_INNER_PWM_PERCENT;
    c.move_left_pwm_percent = S2_MOVE_LEFT_PWM_PERCENT;
    c.move_right_pwm_percent = S2_MOVE_RIGHT_PWM_PERCENT;
    c.move_ramp_pwm_per_s = S2_MOVE_RAMP_PWM_PER_S;
    c.junction_right_pivot_pwm_percent = S2_JUNCTION_RIGHT_PIVOT_PWM_PERCENT;
    c.junction_left_pivot_pwm_percent = S2_JUNCTION_LEFT_PIVOT_PWM_PERCENT;
    c.uturn_left_pwm_percent = S2_UTURN_LEFT_PWM_PERCENT;
    c.uturn_right_pwm_percent = S2_UTURN_RIGHT_PWM_PERCENT;
    c.avoid_heading_kp = S2_AVOID_HEADING_KP;
    c.avoid_pivot_pwm_percent = S2_AVOID_PIVOT_PWM_PERCENT;
    c.avoid_left_turn_ticks_per_degree = S2_AVOID_TURN_TICKS_PER_DEGREE(S2_ENC_LEFT_TICKS_PER_CM);
    c.avoid_right_turn_ticks_per_degree = S2_AVOID_TURN_TICKS_PER_DEGREE(S2_ENC_RIGHT_TICKS_PER_CM);
    return c;
}

/* 1 = reset() configures the controller with alltest_config(). */
static bool all_numbers;

static void reset(MOTION_CONTROLLER *ctl)
{
    MOTION_CONFIG cfg = all_numbers ? alltest_config() : config();
    uturn_r0 = 0;
    frame_report_ms = frame_sequence = 0u;
    armed = halted = turning = configured = false;
    last_imu_sample = imu_count = imu_errors = resumed_at = turn_started = 0;
    /* Controller state added after 2026-10-28 (line-stop debounce/re-arm, S5). */
    line_halt = line_unsafe_active = false;
    line_unsafe_since = rearm_since = 0;
    rearm_l = rearm_r = 0;
    memset(&vc_s5, 0, sizeof(vc_s5));
    /* #48: these tests run with auto-start unless a test turns it off. */
    auto_start = true;
    start_requested = ready_reported = false;
    command_head = command_count = command_polls = 0u;
    /* #72 state. */
    paused = map_received = status_requested = have_imu = in_avoid = false;
    odo_started = false;
    last_letter = 0u;
    last_telemetry = 0u;
    telemetry_error = E_OK;
    distance_cm = 0.0f;
    memset(&last_imu, 0, sizeof(last_imu));
    memset(map_letters, 0, sizeof(map_letters));
    memset(applied_map, 0, sizeof(applied_map));
    has_map = false;
    publish_count = 0u;
    fake_black = fake_white = 0u;
    /* #8: the tests above were written for the S3 position model; the
     * straddle model has its own tests and turns this on itself. */
    straddle = false;
    steer_action = steer_candidate = 0u;
    candidate_since = action_since = clear_since = junction_done = 0u;
    clear_pending = leaving_junction = false;
    pending_letter = 0u;
    turn_stage = turn_cmd = 0u;
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

/* A queued LOST event makes a recoverable line stop. */
static void line_stop(MOTION_CONTROLLER *ctl, uint32_t now)
{
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_LOST << 3u);
    has_event = true;
    robot_control_step(ctl, now, 1, 1);
    EXPECT(halted && line_halt && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
}

/* Stale, searching or paused snapshots must persist for S2_LINE_UNSAFE_MS
 * before the VC stops (one late sample must not latch a stop). */
static void stopped_debounced(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(!halted);
    /* Keep the IMU heartbeat fresh (S2_IMU_STALE_MS) so only the line
     * condition can cause the stop. */
    for (uint32_t t = now + 50u; t < now + S2_LINE_UNSAFE_MS; t += 50u) {
        fake_imu.sample_count++;
        robot_control_step(ctl, t, 1, 1);
        EXPECT(!halted);
    }
    fake_imu.sample_count++;
    stopped(ctl, now + S2_LINE_UNSAFE_MS);
}


/* ---- #8 straddle line model (C and R either side of the line) ---------- */

/* One VC step with a fresh IMU sample, so only the line logic is exercised. */
static void sstep(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    fake_imu.sample_count++;
    robot_control_step(ctl, now, l, r);
}

static void set_sensors(unsigned c_r)   /* C bit 1, R bit 0; S3 in SEARCH */
{
    snapshot = (line_barcode_msg_t){{(uint8_t)c_r, LINE_BARCODE_SEARCH, 0u}};
}

/* Armed and following on both-white, which S3 calls SEARCH. */
static void sfollow(MOTION_CONTROLLER *ctl)
{
    reset(ctl);
    straddle = true;
    set_sensors(0u);
    sstep(ctl, 5000, 0, 0);
    EXPECT(armed && enabled);
    sstep(ctl, 5030, 0, 0);
    EXPECT(!halted && ctl->status.active_command == MOTION_CMD_FOLLOW_LINE &&
           ctl->target_value == 0.0f);
}

static void straddle_tests(void)
{
    MOTION_CONTROLLER ctl;

    /* Both white is "on the line": no stop, even with S3 SEARCH and LOST. */
    sfollow(&ctl);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_LOST << 3u);
    has_event = true;
    for (uint32_t t = 5040u; t <= 7000u; t += 20u) {
        sstep(&ctl, t, 1, 1);
        EXPECT(!halted && ctl.target_value == 0.0f);
    }

    /* Right black: steer right at once (soft), hard after S2_LINE_HARD_MS,
     * and back to straight only after S2_WHITE_CLEAR_MS of white. */
    sfollow(&ctl);
    set_sensors(1u);
    sstep(&ctl, 6000, 1, 1);
    EXPECT(ctl.target_value == 0.5f);
    sstep(&ctl, 6000 + S2_LINE_HARD_MS - 1u, 1, 1);
    EXPECT(ctl.target_value == 0.5f);
    sstep(&ctl, 6000 + S2_LINE_HARD_MS, 1, 1);
    EXPECT(ctl.target_value == 1.0f);
    set_sensors(0u);
    sstep(&ctl, 6200, 1, 1);
    sstep(&ctl, 6200 + S2_WHITE_CLEAR_MS - 1u, 1, 1);
    EXPECT(ctl.target_value == 1.0f);                     /* correction held */
    sstep(&ctl, 6200 + S2_WHITE_CLEAR_MS, 1, 1);
    EXPECT(ctl.target_value == 0.0f);

    /* Centre black: steer left. A reversal must persist S2_LINE_CONFIRM_MS. */
    sfollow(&ctl);
    set_sensors(2u);
    sstep(&ctl, 6000, 1, 1);
    EXPECT(ctl.target_value == -0.5f);
    set_sensors(1u);
    sstep(&ctl, 6010, 1, 1);
    EXPECT(ctl.target_value < 0.0f);                      /* not yet reversed */
    sstep(&ctl, 6010 + S2_LINE_CONFIRM_MS, 1, 1);
    EXPECT(ctl.target_value == 0.5f);

    /* Junction with no barcode: straight on, and the same crossing is
     * ignored until the sensors leave it and S2_JUNCTION_SETTLE_MS passes. */
    sfollow(&ctl);
    set_sensors(3u);
    sstep(&ctl, 6000, 1, 1);
    EXPECT(!turning && leaving_junction && ctl.target_value == 0.0f &&
           ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    /* Still on that crossing: a mapped letter read now must not turn here. */
    send_map("FAZW");
    sstep(&ctl, 6010, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'A' - 'A';
    has_event = true;
    sstep(&ctl, 6020, 1, 1);
    EXPECT(!turning && (pending_letter & 0x80u));        /* kept for the next one */
    set_sensors(0u);
    sstep(&ctl, 6100, 1, 1);
    EXPECT(leaving_junction);
    sstep(&ctl, 6000 + S2_JUNCTION_SETTLE_MS, 1, 1);
    EXPECT(!leaving_junction);
    set_sensors(3u);                                       /* the next crossing */
    sstep(&ctl, 6000 + S2_JUNCTION_SETTLE_MS + 10u, 1, 1);
    EXPECT(turning && turn_cmd == LINE_BARCODE_CMD_RIGHT);

    /* Mapped letter: drive onto the crossing, pivot, then follow again. */
    sfollow(&ctl);
    send_map("FAZW");                       /* slots L,R,S,U: A = right turn */
    sstep(&ctl, 6000, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'A' - 'A';
    has_event = true;
    sstep(&ctl, 6010, 1, 1);
    EXPECT((pending_letter & 0x80u) && last_letter == 1u);
    set_sensors(3u);
    sstep(&ctl, 6020, 1, 1);
    EXPECT(turning && turn_stage == TURN_ADVANCE && !enabled &&
           ctl.status.active_command == MOTION_CMD_FORWARD_CM);
    motion_control_step(&ctl, 6030, 1, 1);
    motion_control_step(&ctl, 6040, 1 + 60, 1 + 60);      /* > 5 cm at 10 ticks/cm */
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    sstep(&ctl, 6050, 61, 61);
    EXPECT(turning && turn_stage == TURN_PIVOT &&
           ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);
    motion_control_step(&ctl, 6060, 61, 61);
    motion_control_step(&ctl, 6070, 61 + 200, 61);        /* left wheel: 90 deg x 2 */
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    sstep(&ctl, 6080, 261, 61);
    EXPECT(!turning && enabled && leaving_junction && !halted);

    /* A letter with no mapping goes straight. */
    sfollow(&ctl);
    send_map("FAZW");
    sstep(&ctl, 6000, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'B' - 'A';
    has_event = true;
    sstep(&ctl, 6010, 1, 1);
    set_sensors(3u);
    sstep(&ctl, 6020, 1, 1);
    EXPECT(!turning && leaving_junction && !(pending_letter & 0x80u));

    /* An S3 fault stops at once and re-arms on a healthy snapshot held
     * still (both white is fine in this model). */
    sfollow(&ctl);
    snapshot.data[1] = LINE_BARCODE_FAULT;
    sstep(&ctl, 6000, 1, 1);
    EXPECT(halted && line_halt && !enabled);
    set_sensors(0u);
    sstep(&ctl, 6100, 1, 1);
    sstep(&ctl, 6100 + S2_LINE_REARM_MS - 101u, 1, 1);
    EXPECT(halted);
    sstep(&ctl, 6000 + S2_LINE_REARM_MS, 1, 1);
    EXPECT(!halted && enabled);

    /* RESUME after PAUSE works on both-white in this model. */
    sfollow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    sstep(&ctl, 6000, 1, 1);
    EXPECT(paused);
    send_command(S1_COMMAND_RESUME);
    sstep(&ctl, 6010, 1, 1);
    EXPECT(!paused && enabled);
}

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-3f)

/* Step the controller every 10 ms, n times, with fixed encoder counts. */
static uint32_t msteps(MOTION_CONTROLLER *m, uint32_t now, int n, int32_t l, int32_t r)
{
    for (int i = 0; i < n; ++i) { now += 10u; motion_control_step(m, now, l, r); }
    return now;
}

/* ---- The rest of all_test, on the motion controller alone --------------- */
static void alltest_motion_tests(void)
{
    MOTION_CONTROLLER m;
    MOTION_CONFIG cfg = alltest_config();
    MOTION_COMMAND follow = {MOTION_CMD_FOLLOW_LINE, 0.0f}, cmd;
    const float L = S2_FOLLOW_LEFT_PWM_PERCENT, R = S2_FOLLOW_RIGHT_PWM_PERCENT;
    const float trim_step = S2_FOLLOW_STRAIGHT_TRIM_RATE * 0.010f;
    const float move_step = S2_MOVE_RAMP_PWM_PER_S * 0.010f;
    const float lt = S2_ENC_LEFT_TICKS_PER_CM, rt = S2_ENC_RIGHT_TICKS_PER_CM;
    uint32_t now;

    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));

    /* Straight trim: right wheel 1.1 cm ahead -> the left wheel gains trim at
     * TRIM_RATE up to KP x (1.1 - deadband); the right wheel is never cut. */
    EXPECT(motion_control_command(&m, &follow, 0u, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, 0u, 100, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, L) && NEAR(m.status.right_pwm_percent, R));
    now = msteps(&m, now, 1, 0, (int32_t)(1.1f * rt));
    EXPECT(NEAR(m.status.left_pwm_percent, L + trim_step) &&
           NEAR(m.status.right_pwm_percent, R));
    now = msteps(&m, now, 200, 0, (int32_t)(1.1f * rt));
    EXPECT(fabsf(m.status.left_pwm_percent -
                 (L + S2_FOLLOW_STRAIGHT_KP * (1.1f - S2_FOLLOW_STRAIGHT_DEADBAND_CM))) < 0.02f);
    /* Far ahead: capped at MAX_TRIM. */
    now = msteps(&m, now, 200, 0, (int32_t)(5.0f * rt));
    EXPECT(NEAR(m.status.left_pwm_percent, L + S2_FOLLOW_STRAIGHT_MAX_TRIM));
    /* Steering ends the stretch; the next one starts from the current ticks,
     * so the old 5 cm difference no longer counts. */
    follow.value = 0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, (int32_t)(5.0f * rt)) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, (int32_t)(5.0f * rt));
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, (int32_t)(5.0f * rt)) == MOTION_RESULT_OK);
    now = msteps(&m, now, 50, 0, (int32_t)(5.0f * rt));
    EXPECT(NEAR(m.status.left_pwm_percent, L) && m.straight_trim == 0.0f);
    /* Left ahead: the right wheel is boosted. Within the deadband: no trim. */
    now = msteps(&m, now, 200, (int32_t)(1.1f * lt), (int32_t)(5.0f * rt));
    EXPECT(NEAR(m.status.left_pwm_percent, L) && m.status.right_pwm_percent > R + 1.0f);
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_STOP, 0}, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 300, 0, (int32_t)(0.05f * rt));
    EXPECT(m.straight_trim == 0.0f);

    /* Correction over: the slowed inner wheel snaps back to cruise at once. */
    follow.value = -1.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(m.status.left_pwm_percent == S2_FOLLOW_HARD_INNER_PWM_PERCENT);
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(m.status.left_pwm_percent >= L - 1e-4f);

    /* Barcode scan: everything at SCAN_SCALE; a left correction keeps the
     * scanner-side wheel at the scan inner PWM instead of stopping it. */
    motion_control_set_scanning(&m, true);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, L * S2_FOLLOW_SCAN_SCALE) &&
           NEAR(m.status.right_pwm_percent, R * S2_FOLLOW_SCAN_SCALE));
    follow.value = -1.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 100, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent,
                S2_FOLLOW_SCAN_LEFT_INNER_PWM_PERCENT * S2_FOLLOW_SCAN_SCALE));
    EXPECT(NEAR(m.status.right_pwm_percent,
                (R + S2_FOLLOW_BOOST_PWM_PERCENT) * S2_FOLLOW_SCAN_SCALE));
    motion_control_set_scanning(&m, false);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(m.status.left_pwm_percent == S2_FOLLOW_HARD_INNER_PWM_PERCENT);
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 100, 0, 0);

    /* Junction advance takes over FOLLOW_LINE with no stop and keeps its
     * PWM; no tolerance (the base 1 cm would end a 0.8 cm move at once). */
    motion_control_set_profile(&m, MOTION_PROFILE_BASE);
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM, 0.8f};
    EXPECT(motion_control_command(&m, &cmd, now, 0, 0) == MOTION_RESULT_BUSY);
    motion_control_set_profile(&m, MOTION_PROFILE_JUNCTION);
    EXPECT(motion_control_command(&m, &cmd, now, 0, 0) == MOTION_RESULT_OK);
    EXPECT(m.status.active_command == MOTION_CMD_FORWARD_CM &&
           NEAR(m.status.left_pwm_percent, L) && NEAR(m.status.right_pwm_percent, R));
    now = msteps(&m, now, 1, 90, 90);                       /* 0.76 cm */
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD &&
           NEAR(m.status.left_pwm_percent, S2_MOVE_LEFT_PWM_PERCENT) &&
           NEAR(m.status.right_pwm_percent, S2_MOVE_RIGHT_PWM_PERCENT));
    now = msteps(&m, now, 1, 95, 95);                       /* 0.80 cm */
    EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);

    /* The pivot starts straight after from the advance's PWM (carry). Right
     * pivot: right wheel stops at once, left at the junction pivot PWM,
     * S2 turn calibration and no 2-degree tolerance. */
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90.0f};
    EXPECT(motion_control_command(&m, &cmd, now + 1u, 95, 95) == MOTION_RESULT_OK);
    EXPECT(NEAR(m.status.left_pwm_percent, S2_MOVE_LEFT_PWM_PERCENT));
    now = msteps(&m, now + 1u, 1, 96, 95);
    EXPECT(m.status.right_pwm_percent == 0.0f &&
           NEAR(m.status.left_pwm_percent, S2_JUNCTION_RIGHT_PIVOT_PWM_PERCENT));
    {
        int32_t target = (int32_t)(90.0f * S2_LEFT_TURN_TICKS_PER_DEGREE);
        now = msteps(&m, now, 1, 95 + target - 5, 95);
        EXPECT(m.status.state == MOTION_STATE_TURNING_RIGHT);
        now = msteps(&m, now, 1, 95 + target + 1, 95);
        EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);
    }
    /* Left pivot drives the right wheel at its own PWM. Started late (no
     * carry), it ramps from rest. */
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_LEFT_DEG, 90.0f};
    EXPECT(motion_control_command(&m, &cmd, now + 200u, 0, 0) == MOTION_RESULT_OK);
    EXPECT(m.status.right_pwm_percent == 0.0f);
    now = msteps(&m, now + 200u, 1, 0, 1);
    EXPECT(NEAR(m.status.right_pwm_percent, move_step) && m.status.left_pwm_percent == 0.0f);
    for (int i = 2; i <= 30; ++i) now = msteps(&m, now, 1, 0, i);
    EXPECT(NEAR(m.status.right_pwm_percent, S2_JUNCTION_LEFT_PIVOT_PWM_PERCENT));
    {
        int32_t target = (int32_t)(90.0f * S2_RIGHT_TURN_TICKS_PER_DEGREE);
        now = msteps(&m, now, 1, 0, target - 5);
        EXPECT(m.status.state == MOTION_STATE_TURNING_LEFT);   /* no 2-degree tolerance */
        now = msteps(&m, now, 1, 0, target + 1);
        EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);
    }

    /* U-turn: counter-rotates, never ends by itself, ends on STOP. */
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    motion_control_set_profile(&m, MOTION_PROFILE_UTURN);
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 180.0f};
    EXPECT(motion_control_command(&m, &cmd, 0u, 0, 0) == MOTION_RESULT_INVALID_COMMAND);
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_LEFT_DEG, 180.0f};
    EXPECT(motion_control_command(&m, &cmd, 0u, 0, 0) == MOTION_RESULT_OK);
    now = 0u;
    for (int i = 1; i <= 300; ++i) now = msteps(&m, now, 1, -i * 10, i * 10);
    EXPECT(m.status.state == MOTION_STATE_TURNING_LEFT &&
           NEAR(m.status.left_pwm_percent, S2_UTURN_LEFT_PWM_PERCENT) &&
           NEAR(m.status.right_pwm_percent, S2_UTURN_RIGHT_PWM_PERCENT));
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_STOP, 0}, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, -3000, 3000);
    EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.left_pwm_percent == 0.0f);
    /* ... but a stalled right wheel still faults. */
    EXPECT(motion_control_command(&m, &cmd, now, -3000, 3000) == MOTION_RESULT_OK);
    now = msteps(&m, now, 60, -3000, 3000);
    EXPECT(m.status.state == MOTION_STATE_FAULT);

    /* S5 leg: heading held by cutting the leading wheel KP per cm; no stall
     * fault (a cut wheel may stand still); no tolerance. */
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    motion_control_set_profile(&m, MOTION_PROFILE_AVOID);
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM, 10.0f};
    EXPECT(motion_control_command(&m, &cmd, 0u, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, 0u, 1, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, move_step) && NEAR(m.status.right_pwm_percent, move_step));
    now = msteps(&m, now, 100, 0, 0);                       /* 1 s still: no fault */
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD &&
           NEAR(m.status.left_pwm_percent, S2_MOVE_LEFT_PWM_PERCENT) &&
           NEAR(m.status.right_pwm_percent, S2_MOVE_RIGHT_PWM_PERCENT));
    now = msteps(&m, now, 100, (int32_t)(2.0f * lt), (int32_t)(2.5f * rt));
    EXPECT(fabsf(m.status.right_pwm_percent -
                 (S2_MOVE_RIGHT_PWM_PERCENT - S2_AVOID_HEADING_KP * 0.5f)) < 0.05f &&
           NEAR(m.status.left_pwm_percent, S2_MOVE_LEFT_PWM_PERCENT));
    now = msteps(&m, now, 1, (int32_t)(9.95f * lt), (int32_t)(9.95f * rt));
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD);
    now = msteps(&m, now, 1, (int32_t)(10.01f * lt), (int32_t)(10.01f * rt));
    EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);
    /* S5 pivot: 20 % on the moving wheel, S5's pivot-radius calibration. */
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90.0f};
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_STOP, 0}, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, 0);
    EXPECT(motion_control_command(&m, &cmd, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 1, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, move_step));     /* from rest after STOP */
    now = msteps(&m, now, 50, 50, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, S2_AVOID_PIVOT_PWM_PERCENT));
    {
        float tpd = S2_AVOID_TURN_TICKS_PER_DEGREE(S2_ENC_LEFT_TICKS_PER_CM);
        now = msteps(&m, now, 1, (int32_t)(90.0f * tpd) - 5, 0);
        EXPECT(m.status.state == MOTION_STATE_TURNING_RIGHT);   /* S2 calibration would be done */
        now = msteps(&m, now, 1, (int32_t)(90.0f * tpd) + 1, 0);
        EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);
    }

    /* Invalid profile numbers are rejected. */
    cfg.uturn_right_pwm_percent = 0.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg = alltest_config();
    cfg.uturn_left_pwm_percent = -101.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg = alltest_config();
    cfg.follow_scan_scale = 1.5f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg = alltest_config();
    cfg.avoid_left_turn_ticks_per_degree = 0.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg = alltest_config();
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
}

/* ---- ... and through the VC (straddle model, integrated numbers) ------- */
static void alltest_robot_tests(void)
{
    MOTION_CONTROLLER ctl;
    uint32_t now;

    all_numbers = true;

    /* The scan flag follows GP28 (scanner) or an open capture. */
    sfollow(&ctl);
    snapshot = (line_barcode_msg_t){{LINE_BARCODE_LEFT, LINE_BARCODE_SEARCH, 0u}};
    sstep(&ctl, 5040, 0, 0);
    EXPECT(ctl.scanning);
    snapshot = (line_barcode_msg_t){{0u, LINE_BARCODE_SEARCH | LINE_BARCODE_CAPTURE_ACTIVE, 0u}};
    sstep(&ctl, 5050, 0, 0);
    EXPECT(ctl.scanning);
    set_sensors(0u);
    sstep(&ctl, 5060, 0, 0);
    EXPECT(!ctl.scanning);

    /* Mapped U-turn: no stop between following and the advance, then the
     * counter-rotating spin ends only once the right wheel has done half its
     * pivot arc AND the centre sensor is on black. */
    for (int centre_late = 0; centre_late < 2; ++centre_late) {
        int32_t half = (int32_t)(180.0f * S2_RIGHT_TURN_TICKS_PER_DEGREE * 0.5f);
        sfollow(&ctl);
        send_map("FAZW");                                   /* W = U-turn */
        now = msteps(&ctl, 5030, 50, 0, 0);                 /* cruising */
        queued = snapshot;
        queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
        queued.data[2] = 'W' - 'A';
        has_event = true;
        sstep(&ctl, now, 0, 0);
        set_sensors(3u);
        sstep(&ctl, now + 1u, 0, 0);
        EXPECT(turning && turn_stage == TURN_ADVANCE && turn_cmd == LINE_BARCODE_CMD_UTURN);
        EXPECT(ctl.status.active_command == MOTION_CMD_FORWARD_CM &&
               ctl.status.left_pwm_percent > 10.0f);        /* not stopped first */
        now = msteps(&ctl, now + 1u, 1, 100, 100);          /* > 0.8 cm */
        EXPECT(ctl.status.state == MOTION_STATE_IDLE);
        sstep(&ctl, now + 1u, 100, 100);
        EXPECT(turn_stage == TURN_PIVOT && ctl.active_profile == MOTION_PROFILE_UTURN &&
               ctl.status.left_pwm_percent > 10.0f);        /* carried on */
        now = msteps(&ctl, now + 1u, 30, 100 - 300, 100 + 300);
        EXPECT(ctl.status.left_pwm_percent < 0.0f);         /* reversing */
        set_sensors(centre_late ? 0u : 2u);                 /* centre black early */
        sstep(&ctl, now, 100 - half + 100, 100 + half - 100);
        EXPECT(turning);                                    /* arc not done yet */
        set_sensors(0u);
        sstep(&ctl, now + 1u, 100 - half, 100 + half + 1);
        EXPECT(turning);                                    /* arc done, centre white */
        set_sensors(2u);
        sstep(&ctl, now + 2u, 100 - half, 100 + half + 2);
        EXPECT(!turning && !halted && enabled && leaving_junction);
        EXPECT(ctl.status.left_pwm_percent == 0.0f);        /* spin stopped */
    }

    /* Right turn: the pivot uses the junction profile. */
    sfollow(&ctl);
    send_map("FAZW");
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'A' - 'A';
    has_event = true;
    sstep(&ctl, 5040, 0, 0);
    set_sensors(3u);
    sstep(&ctl, 5050, 0, 0);
    motion_control_step(&ctl, 5060, 100, 100);
    sstep(&ctl, 5061, 100, 100);
    EXPECT(turn_stage == TURN_PIVOT && ctl.active_profile == MOTION_PROFILE_JUNCTION &&
           ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);

    /* S5: moves use the AVOID profile, all_test's settle and timeout, and
     * handing back gives the rejoin the same grace as a junction exit. */
    EXPECT(vc_s5_ops.hold_settle_ms == S2_AVOID_HOLD_SETTLE_MS);
    sfollow(&ctl);
    {
        VC_CTX c = {&ctl, 6000u, 0, 0};
        S5_REQUEST req = {0};
        req.move = S5_MOVE_FWD;
        req.value = 30u;
        vc_stop(&c);
        EXPECT(vc_start_move(&c, &req));
        EXPECT(ctl.active_profile == MOTION_PROFILE_AVOID &&
               ctl.status.active_command == MOTION_CMD_FORWARD_CM);
        steer_action = 1u;
        EXPECT(vc_give_back(&c));
        EXPECT(leaving_junction && junction_done == 6000u && steer_action == 0u);
    }

    all_numbers = false;
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
    /* Tilt is report-only (#67): the robot keeps following. */
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_TILT;
    fake_imu.event = IMU_EVENT_TILT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!halted && enabled);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    follow(&ctl); fake_imu.event = IMU_EVENT_FAULT; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.sensor_ok = FALSE; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.read_errors++; stopped(&ctl, 5041);
    follow(&ctl); stopped(&ctl, 5101); /* stale IMU heartbeat */
    follow(&ctl); snapshot_error = E_TMOUT; stopped_debounced(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_FAULT; stopped(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_SEARCH; stopped_debounced(&ctl, 5041);
    for (unsigned event = 0; event < 2; ++event) {
        const unsigned events[] = {LINE_BARCODE_EVT_LOST, LINE_BARCODE_EVT_FAULT};
        follow(&ctl); queued = snapshot;
        queued.data[1] = (uint8_t)(events[event] << 3u); has_event = true;
        stopped(&ctl, 5041);
    }
    /* #72 (decided 21 Nov): a junction with no mapped barcode before it goes
     * straight on, both before and after an MQTT map has arrived. */
    for (unsigned with_map = 0; with_map < 2; ++with_map) {
        follow(&ctl); queued = snapshot;
        queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_JUNCTION << 3u); has_event = true;
        map_received = with_map != 0u;
        robot_control_step(&ctl, 5041, 1, 1);
        EXPECT(!halted && enabled && !turning);
        robot_control_step(&ctl, 5042, 1, 1);
        motion_control_step(&ctl, 5050, 2, 2);
        EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    }
    /* Line stops re-arm by themselves after S2_LINE_REARM_MS of stable black
     * on centre + right with the wheels still (ported from the retired
     * all_buddies host tests, #9). */
    {
        const uint32_t t0 = 5041u, rearm = S2_LINE_REARM_MS;
        follow(&ctl); line_stop(&ctl, t0);
        robot_control_step(&ctl, t0 + 50u, 1, 1);
        robot_control_step(&ctl, t0 + rearm - 1u, 1, 1);
        EXPECT(halted && !enabled);                 /* not yet */
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(!halted && enabled);                 /* re-armed on its own */

        /* A brief loss of black restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        snapshot.data[0] = 0u; robot_control_step(&ctl, 5500u, 1, 1);
        snapshot.data[0] = 3u | (2u << 3u);
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(halted);
        robot_control_step(&ctl, 5500u + rearm, 1, 1);
        EXPECT(!halted && enabled);

        /* Moving the robot by hand (encoders change) restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        robot_control_step(&ctl, 5500u, 5, 5);
        robot_control_step(&ctl, t0 + rearm, 5, 5);
        EXPECT(halted);
        robot_control_step(&ctl, 5500u + rearm, 5, 5);
        EXPECT(!halted && enabled);

        /* An IMU (safety) stop never re-arms, even on stable black. */
        follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
        robot_control_step(&ctl, t0, 1, 1);
        EXPECT(halted && !line_halt);
        fake_imu.event = IMU_EVENT_NONE;
        for (uint32_t t = t0 + 100u; t <= t0 + 3u * rearm; t += 100u) {
            fake_imu.sample_count++;
            robot_control_step(&ctl, t, 1, 1);
        }
        EXPECT(halted && ctl.status.left_pwm_percent == 0);
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
    /* #48 Ready state: with auto-start off, an initialised robot waits,
     * motors off, until START. Other commands do not start it. */
    reset(&ctl);
    auto_start = false;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!armed && !enabled && ready_reported);
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5050, 0, 0);
    EXPECT(!armed && !enabled && command_count == 0u); /* drained, not a start */
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    send_command(S1_COMMAND_START);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 0, 0);
    EXPECT(armed && enabled && start_requested);
    robot_control_step(&ctl, 5130, 0, 0);
    motion_control_step(&ctl, 5140, 1, 1);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    send_command(S1_COMMAND_START);                       /* already running */
    robot_control_step(&ctl, 5150, 1, 1);
    EXPECT(armed && command_count == 0u);

    /* A START sent during initialisation is kept until the robot is Ready. */
    reset(&ctl);
    auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);                  /* before S2_ROBOT_START_MS */
    EXPECT(!armed && start_requested && !ready_reported);
    fake_imu.sample_count++;
    robot_control_step(&ctl, S2_ROBOT_START_MS, 0, 0);
    EXPECT(armed && enabled);

    /* Polling never blocks and drains at most S1_QUEUE_DEPTH per step. */
    reset(&ctl);
    auto_start = false;
    for (unsigned i = 0u; i < 10u; ++i) send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 1000, 0, 0);
    EXPECT(command_count == 10u - S1_QUEUE_DEPTH && command_polls == S1_QUEUE_DEPTH);

    /* ---- #72 MQTT link ------------------------------------------------- */

    /* PAUSE stops at once and holds; RESUME on a healthy line and IMU
     * restarts line following. */
    follow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(paused && !enabled && !halted);
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    for (uint32_t t = 5100u; t <= 7000u; t += 100u) {
        fake_imu.sample_count++;
        robot_control_step(&ctl, t, 1, 1);
        motion_control_step(&ctl, t, 1, 1);
        EXPECT(paused && ctl.status.left_pwm_percent == 0);
    }
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 7050, 1, 1);
    EXPECT(!paused && enabled);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 7100, 1, 1);
    motion_control_step(&ctl, 7110, 2, 2);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    /* RESUME is refused off the line or with an unhealthy IMU. */
    follow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 5041, 1, 1);
    snapshot.data[0] = 0u;                                 /* off the line */
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5060, 1, 1);
    EXPECT(paused && !enabled);
    snapshot.data[0] = 3u | (2u << 3u);
    fake_imu.sensor_ok = FALSE;
    send_command(S1_COMMAND_RESUME);
    robot_control_step(&ctl, 5070, 1, 1);
    EXPECT(paused && !enabled);

    /* PAUSE during a line stop blocks the automatic re-arm until RESUME. */
    {
        const uint32_t t0 = 5041u, rearm = S2_LINE_REARM_MS;
        follow(&ctl); line_stop(&ctl, t0);
        send_command(S1_COMMAND_PAUSE);
        robot_control_step(&ctl, t0 + 10u, 1, 1);
        robot_control_step(&ctl, t0 + 3u * rearm, 1, 1);
        EXPECT(halted && paused && !enabled);              /* no re-arm while paused */
        send_command(S1_COMMAND_RESUME);
        fake_imu.sample_count++;
        robot_control_step(&ctl, t0 + 3u * rearm + 1u, 1, 1);
        EXPECT(!paused && halted);          /* time paused doesn't count */
        robot_control_step(&ctl, t0 + 4u * rearm, 1, 1);
        EXPECT(halted && !enabled);
        robot_control_step(&ctl, t0 + 4u * rearm + 1u, 1, 1);
        EXPECT(!halted && enabled);         /* a full re-arm wait after RESUME */
    }

    /* A safety stop ignores PAUSE and RESUME: it needs a reboot. */
    follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(halted && !line_halt);
    fake_imu.event = IMU_EVENT_NONE;
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 1, 1);
    EXPECT(halted && !line_halt && !paused && !enabled);

    /* PAUSE in Ready cancels a pending START. */
    reset(&ctl);
    auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 1100, 0, 0);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!armed && !start_requested);

    /* The MQTT letter map is applied to S3; a later barcode and its mapped
     * direction appear in telemetry. A bad map changes nothing. */
    follow(&ctl);
    EXPECT(applied_map[0] == '\0' && !map_received);        /* no built-in map */
    send_map("FAZW");
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(map_received && strcmp(applied_map, "FAZW") == 0);
    pending_map.bytes[0] = pending_map.bytes[1] = pending_map.bytes[2] = 0u; /* A,A,A,A */
    has_map = true;
    robot_control_step(&ctl, 5042, 1, 1);
    EXPECT(strcmp(applied_map, "FAZW") == 0);               /* duplicates rejected */
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'Z' - 'A';
    has_event = true;
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5060, 1, 1);
    EXPECT(last_letter == 26u);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5061, 1, 1);
    EXPECT(field(S1_FIELD_BARCODE_LETTER) == 26 &&
           field(S1_FIELD_BARCODE) == S1_BARCODE_STRAIGHT); /* Z = straight */
    EXPECT(field(S1_FIELD_MAP_RECEIVED) == 1);             /* #38 */

    /* Telemetry: once a second, STATUS forces one at once; state, line bits,
     * distance, hump and barcode counts come from the live sources. */
    reset(&ctl);
    auto_start = false;
    robot_control_step(&ctl, 1000, 0, 0);
    EXPECT(publish_count == 1u && field(S1_FIELD_STATE) == S1_STATE_IDLE);
    EXPECT(field(S1_FIELD_MAP_RECEIVED) == 0);             /* #38: waiting for map */
    robot_control_step(&ctl, 1500, 0, 0);
    EXPECT(publish_count == 1u);
    robot_control_step(&ctl, 2000, 0, 0);
    EXPECT(publish_count == 2u);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 2001, 0, 0);
    EXPECT(publish_count == 3u);
    follow(&ctl);
    fake_imu.ramp_height_mm = 23u;
    fake_black = 15u; fake_white = 14u;
    snapshot.data[0] = 6u | (2u << 3u);                    /* S3 L+C black */
    fake_imu.sample_count++;
    robot_control_step(&ctl, 6000, 100, 100);              /* 10 ticks/cm: 10 cm */
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6001, 100, 100);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_FOLLOWING);
    EXPECT(field(S1_FIELD_LINE_SENSORS) == 3);             /* wire: L bit0, C bit1 */
    EXPECT(field(S1_FIELD_DISTANCE_CM) == 10);
    EXPECT(field(S1_FIELD_HIGHEST_HUMP_MM) == 23);
    EXPECT(field(S1_FIELD_BARCODE_BLACK) == 15 && field(S1_FIELD_BARCODE_WHITE) == 14);
    EXPECT(field(S1_FIELD_FAULTS) == 0);
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6002, 100, 100);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_IDLE);        /* paused */
    follow(&ctl); line_stop(&ctl, 5041);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5042, 1, 1);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_RECOVERING);
    follow(&ctl); fake_imu.sensor_ok = FALSE;
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_FAULT &&
           (field(S1_FIELD_FAULTS) & S1_FAULT_IMU) != 0);

    follow_tuning_tests();   /* #8 */
    straddle_tests();        /* #8 */
    alltest_motion_tests();  /* #8: the rest of all_test */
    alltest_robot_tests();

    printf("robot host tests: %u checks passed\n", checks);
    return 0;
}

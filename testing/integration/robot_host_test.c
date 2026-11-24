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
static unsigned command_head, command_count, command_polls;
#if !S1_REAL_LINK
static unsigned char commands[16];
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
#else
/* INT-15 (int/bad_commands, #51): the REAL S1 receive path. Commands arrive
 * as broker bytes in mqtt_manager.c's callbacks, go through s1_deliver_command
 * and the real command buffer (subsystem1.c), and the VC reads them with the
 * real subsystem1_receive_command. Telemetry and the letter map keep the
 * fakes below, so the real versions are renamed out of the way. */
#define subsystem1_publish_telemetry real_subsystem1_publish_telemetry
#define subsystem1_receive_mapping real_subsystem1_receive_mapping
#include "../../app_program/subsystem1/subsystem1.c"
#undef subsystem1_publish_telemetry
#undef subsystem1_receive_mapping
#include "../../app_program/subsystem1/mqtt_manager.c"

/* Message buffers, sized as the kernel sizes them: each message takes
 * sizeof(INT) for its length plus its payload rounded up to sizeof(INT),
 * out of bufsz. So the command buffer holds S1_QUEUE_DEPTH messages. */
#define FAKE_MBFS 4
#define FAKE_MBF_MSGS 16
/* Largest message: 19-byte telemetry since #41 (was 15). */
#define FAKE_MBF_MAXMSZ 32
static struct {
    INT bufsz, maxmsz, used, head, count;
    unsigned char msg[FAKE_MBF_MSGS][FAKE_MBF_MAXMSZ];
    INT size[FAKE_MBF_MSGS];
} mbf[FAKE_MBFS];
static ID mbf_count;
static INT mbf_cost(INT size)
{ return (INT)sizeof(INT) + ((size + (INT)sizeof(INT) - 1) / (INT)sizeof(INT)) * (INT)sizeof(INT); }
ID tk_cre_mbf(const T_CMBF *pk)
{
    assert(mbf_count < FAKE_MBFS && pk->maxmsz <= FAKE_MBF_MAXMSZ);
    memset(&mbf[mbf_count], 0, sizeof(mbf[0]));
    mbf[mbf_count].bufsz = pk->bufsz; mbf[mbf_count].maxmsz = pk->maxmsz;
    return ++mbf_count;
}
ER tk_del_mbf(ID id) { (void)id; return E_OK; }
ER tk_snd_mbf(ID id, const void *msg, INT size, TMO tmout)
{
    assert(id >= 1 && id <= mbf_count && tmout == TMO_POL);
    if (size <= 0 || size > mbf[id - 1].maxmsz) return E_PAR;
    if (mbf[id - 1].used + mbf_cost(size) > mbf[id - 1].bufsz) return E_TMOUT;
    unsigned slot = (unsigned)(mbf[id - 1].head + mbf[id - 1].count) % FAKE_MBF_MSGS;
    memcpy(mbf[id - 1].msg[slot], msg, (size_t)size);
    mbf[id - 1].size[slot] = size;
    mbf[id - 1].used += mbf_cost(size);
    ++mbf[id - 1].count;
    if (id == g_command_queue) ++command_count;
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmout)
{
    assert(id >= 1 && id <= mbf_count && tmout == TMO_POL);
    if (id == g_command_queue) ++command_polls;
    if (mbf[id - 1].count == 0) return E_TMOUT;
    unsigned slot = (unsigned)mbf[id - 1].head % FAKE_MBF_MSGS;
    INT size = mbf[id - 1].size[slot];
    memcpy(msg, mbf[id - 1].msg[slot], (size_t)size);
    mbf[id - 1].head = (mbf[id - 1].head + 1) % FAKE_MBF_MSGS;
    --mbf[id - 1].count;
    mbf[id - 1].used -= mbf_cost(size);
    if (id == g_command_queue) --command_count;
    return size;
}

/* lwIP: one fake client. Connecting captures the callbacks mqtt_manager.c
 * installs; the test then plays the broker through them. */
static struct mqtt_client_s { int connected; } fake_client;
static mqtt_connection_cb_t mqtt_conn_cb;
static mqtt_incoming_publish_cb_t mqtt_pub_cb;
static mqtt_incoming_data_cb_t mqtt_data_cb;
static char subscribed[2][40];
static unsigned subscribe_count, subscribe_qos_ok;
int ip4addr_aton(const char *cp, ip4_addr_t *addr) { (void)cp; addr->addr = 1u; return 1; }
mqtt_client_t *mqtt_client_new(void) { return &fake_client; }
err_t mqtt_client_connect(mqtt_client_t *c, const ip_addr_t *ip, u16_t port,
                          mqtt_connection_cb_t cb, void *arg,
                          const struct mqtt_connect_client_info_t *info)
{ (void)c; (void)ip; (void)port; (void)arg; (void)info; mqtt_conn_cb = cb; return ERR_OK; }
void mqtt_disconnect(mqtt_client_t *c) { c->connected = 0; }
u8_t mqtt_client_is_connected(mqtt_client_t *c) { return (u8_t)c->connected; }
void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t pub,
                             mqtt_incoming_data_cb_t data, void *arg)
{ (void)c; (void)arg; mqtt_pub_cb = pub; mqtt_data_cb = data; }
err_t mqtt_subscribe(mqtt_client_t *c, const char *topic, u8_t qos,
                     mqtt_request_cb_t cb, void *arg)
{
    (void)c; (void)cb; (void)arg;
    if (subscribe_count < 2u) snprintf(subscribed[subscribe_count], 40, "%s", topic);
    ++subscribe_count; subscribe_qos_ok += qos == 1u;
    return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload,
                   u16_t len, u8_t qos, u8_t retain, mqtt_request_cb_t cb, void *arg)
{ (void)c; (void)topic; (void)payload; (void)len; (void)qos; (void)retain; (void)cb; (void)arg; return ERR_OK; }
void sys_timeout(u32_t ms, sys_timeout_handler h, void *arg) { (void)ms; (void)h; (void)arg; }
void sys_untimeout(sys_timeout_handler h, void *arg) { (void)h; (void)arg; }
u32_t sys_now(void) { return 0u; }

/* The broker delivers one PUBLISH: announced length, then the payload in
 * chunks of `chunk` bytes (0 = one chunk), the last flagged. */
static void broker_publish(const char *topic, const unsigned char *data,
                           unsigned announced, unsigned len, unsigned chunk)
{
    mqtt_pub_cb(NULL, topic, announced);
    if (len == 0u) { mqtt_data_cb(NULL, data, 0u, MQTT_DATA_FLAG_LAST); return; }
    if (chunk == 0u) chunk = len;
    for (unsigned at = 0u; at < len; at += chunk) {
        unsigned n = len - at < chunk ? len - at : chunk;
        mqtt_data_cb(NULL, data + at, (u16_t)n, at + n >= len ? MQTT_DATA_FLAG_LAST : 0u);
    }
}
static void send_command(unsigned code)
{
    unsigned char byte = (unsigned char)code;
    broker_publish(MQTT_CLIENT_ID "/command", &byte, 1u, 1u, 0u);
}
/* Bring the real S1 link up once: queues, then an accepted connection. */
static void s1_link_up(void)
{
    if (mqtt_pub_cb != NULL) return;
    assert(subsystem1_init() == E_OK);
    fake_client.connected = 0;
    mqtt_utk_start();
    assert(mqtt_conn_cb != NULL);
    fake_client.connected = 1;
    mqtt_conn_cb(&fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    assert(mqtt_pub_cb != NULL && mqtt_data_cb != NULL);
}
static void s1_link_reset(void)
{
    s1_link_up();
    for (ID i = 0; i < mbf_count; ++i) { mbf[i].used = mbf[i].head = mbf[i].count = 0; }
    memset(&g_command_stats, 0, sizeof(g_command_stats));
    g_command_stats_reported = g_command_stats;
    g_command_log_budget = S1_COMMAND_LOG_BURST;
}
#endif

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

#if S5_REAL_EXECUTOR
/* INT-13 (int/robot_avoidance): the REAL S5 executor (s5_executor.c) runs
 * behind the VC. Only its kernel boundary is faked: the event flag
 * g_flg_s5_evt, and the request word S5's task would publish. The word is
 * packed and unpacked by the real s5_request_pack/unpack (obstacle_logic.c),
 * exactly as task_obstacle.c's subsystem5_get_request() does. */
#include "../../app_program/subsystem5/obstacle_logic.h"
ID g_flg_s5_evt = 42;
static UINT s5_flags;   /* bits set in g_flg_s5_evt */
static UW s5_word;      /* S5's published request word */
static UB s5_seq;
ER tk_set_flg(ID flgid, UINT setptn)
{ assert(flgid == g_flg_s5_evt); s5_flags |= setptn; return E_OK; }
ER tk_clr_flg(ID flgid, UINT clrptn)
{ assert(flgid == g_flg_s5_evt); s5_flags &= clrptn; return E_OK; }
ER subsystem5_get_request(S5_REQUEST *p_req)
{
    if (p_req == NULL) return E_PAR;
    return s5_request_unpack(s5_word, p_req) ? E_OK : E_PAR;
}
/* What S5's s5_post_line() does: next seq, one store. */
static void s5_post(UB kind, UB move, UH value, BOOL stop_on_line)
{
    S5_REQUEST req = {kind, move, 0u, stop_on_line, value, FALSE, FALSE, FALSE};
    s5_seq = (UB)((s5_seq + 1u) & S5_REQ_SEQ_MASK);
    req.seq = s5_seq;
    s5_word = s5_request_pack(&req);
}
#else
/* S5 executor: modelled as idle (no obstacle, no start-up hold), so these
 * tests exercise the VC's own line/barcode/IMU logic. The avoidance
 * hand-over runs against the real executor in int/robot_avoidance (INT-13:
 * this file built with S5_REAL_EXECUTOR=1). */
static unsigned s5_steps;
void s5_exec_set_following(S5_EXEC *p_x, BOOL b_following)
{ (void)p_x; (void)b_following; }
BOOL s5_exec_startup_hold(void) { return FALSE; }
#endif
/* #40: S5's last obstacle scan, as subsystem5_get_obstacle() returns it. */
static S5_OBSTACLE_INFO fake_obstacle;
static ER fake_obstacle_er = E_OK;
ER subsystem5_get_obstacle(S5_OBSTACLE_INFO *p_info)
{
    if (fake_obstacle_er != E_OK) { memset(p_info, 0, sizeof(*p_info)); return fake_obstacle_er; }
    *p_info = fake_obstacle; return E_OK;
}
/* #46: S5's safe-stop fault bits, as subsystem5_get_faults() returns them. */
static UW fake_s5_faults;
UW subsystem5_get_faults(void) { return fake_s5_faults; }
#if !S5_REAL_EXECUTOR
BOOL s5_exec_step(S5_EXEC *p_x, const S5_EXEC_OPS *p_ops, void *p_ctx, UW now_ms)
{ (void)p_x; (void)p_ops; (void)p_ctx; (void)now_ms; ++s5_steps; return FALSE; }
#endif

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
    c.avoid_spin_pwm_percent = S2_AVOID_SPIN_PWM_PERCENT;
    c.avoid_left_turn_ticks_per_degree = S2_AVOID_LEFT_TURN_TICKS_PER_DEGREE;
    c.avoid_right_turn_ticks_per_degree = S2_AVOID_RIGHT_TURN_TICKS_PER_DEGREE;
    return c;
}

/* 1 = reset() configures the controller with alltest_config(). */
static bool all_numbers;

static void reset(MOTION_CONTROLLER *ctl)
{
    MOTION_CONFIG cfg = all_numbers ? alltest_config() : config();
    g_uturn_r0 = 0;
    g_frame_report_ms = g_frame_sequence = 0u;
    g_b_armed = g_b_halted = g_b_turning = g_b_configured = false;
    g_last_imu_sample = g_imu_count = g_imu_errors = g_resumed_at = g_turn_started = 0;
    /* Controller state added after 2026-10-28 (line-stop debounce/re-arm, S5). */
    g_b_line_halt = g_b_line_unsafe_active = false;
    g_line_unsafe_since = g_rearm_since = 0;
    g_rearm_l = g_rearm_r = 0;
    memset(&g_vc_s5, 0, sizeof(g_vc_s5));
    /* #48: these tests run with auto-start unless a test turns it off. */
    g_b_auto_start = true;
    g_b_start_requested = g_b_ready_reported = false;
    command_head = command_count = command_polls = 0u;
#if S1_REAL_LINK
    s1_link_reset();
#endif
    /* #72 state. */
    g_b_paused = g_b_map_received = g_b_status_requested = g_b_have_imu = g_b_in_avoid = false;
    g_b_odo_started = false;
    g_last_letter = 0u;
    g_last_telemetry = 0u;
    g_telemetry_error = E_OK;
    g_distance_cm = 0.0f;
    memset(&g_last_imu, 0, sizeof(g_last_imu));
    memset(g_map_letters, 0, sizeof(g_map_letters));
    memset(applied_map, 0, sizeof(applied_map));
    has_map = false;
    publish_count = 0u;
    fake_black = fake_white = 0u;
    /* #8: the tests above were written for the S3 position model; the
     * straddle model has its own tests and turns this on itself. */
    g_b_straddle = false;
    g_steer_action = g_steer_candidate = 0u;
    g_candidate_since = g_action_since = g_clear_since = g_junction_done = 0u;
    g_b_clear_pending = g_b_leaving_junction = false;
    g_pending_letter = 0u;
    g_b_lost_watch = g_b_lost_probe = g_b_lost_stop = g_b_lost_seen_black = false;
    g_lost_side = 0u;
    g_lost_l0 = g_lost_r0 = 0;
    g_lost_last_step = g_lost_max_mm = 0u;
    g_turn_stage = g_turn_cmd = 0u;
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
    EXPECT(g_b_armed && enabled);
    EXPECT(ctl->status.state == MOTION_STATE_IDLE);
    robot_control_step(ctl, 5030, 0, 0);
    motion_control_step(ctl, 5040, 1, 1);
    EXPECT(ctl->status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT(ctl->status.left_pwm_percent > 0);
}

static void stopped(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(g_b_halted && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
    fake_imu.motion_state = IMU_MOTION_NORMAL;
    fake_imu.sensor_ok = TRUE;
    fake_imu.sample_count++;
    robot_control_step(ctl, now + 1, 1, 1);
    EXPECT(g_b_halted && ctl->status.left_pwm_percent == 0);
}

/* A queued LOST event makes a recoverable line stop. */
static void line_stop(MOTION_CONTROLLER *ctl, uint32_t now)
{
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_LOST << 3u);
    has_event = true;
    robot_control_step(ctl, now, 1, 1);
    EXPECT(g_b_halted && g_b_line_halt && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
}

/* Stale, searching or paused snapshots must persist for S2_LINE_UNSAFE_MS
 * before the VC stops (one late sample must not latch a stop). */
static void stopped_debounced(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(!g_b_halted);
    /* Keep the IMU heartbeat fresh (S2_IMU_STALE_MS) so only the line
     * condition can cause the stop. */
    for (uint32_t t = now + 50u; t < now + S2_LINE_UNSAFE_MS; t += 50u) {
        fake_imu.sample_count++;
        robot_control_step(ctl, t, 1, 1);
        EXPECT(!g_b_halted);
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
    g_b_straddle = true;
    set_sensors(0u);
    sstep(ctl, 5000, 0, 0);
    EXPECT(g_b_armed && enabled);
    sstep(ctl, 5030, 0, 0);
    EXPECT(!g_b_halted && ctl->status.active_command == MOTION_CMD_FOLLOW_LINE &&
           ctl->target_value == 0.0f);
}

/* Encoder ticks (same on both wheels) for this many mm of travel. */
static int32_t ticks_for_mm(float mm)
{
    return (int32_t)(mm / 10.0f /
                     (0.5f * (1.0f / S2_ENC_LEFT_TICKS_PER_CM +
                              1.0f / S2_ENC_RIGHT_TICKS_PER_CM)) + 0.5f);
}

/* Drive straight on (both wheels) in 10 ms steps until *ticks reaches
   `to`, `per` ticks a step. Returns the new time. */
static uint32_t sdrive(MOTION_CONTROLLER *ctl, uint32_t t, int32_t *ticks,
                       int32_t to, int32_t per)
{
    while (*ticks < to) {
        t += 10u;
        *ticks = *ticks + per > to ? to : *ticks + per;
        sstep(ctl, t, *ticks, *ticks);
    }
    return t;
}

/* Following, then R black (steer right) for `hold_ms`, then white.
   Returns the time of the first white step; *ticks is the tick count. */
static uint32_t scorrect_on(MOTION_CONTROLLER *ctl, unsigned black,
                            uint32_t hold_ms, int32_t *ticks)
{
    uint32_t t = 6000u;
    sfollow(ctl);
    *ticks = 1;
    set_sensors(black);
    sstep(ctl, t, *ticks, *ticks);
    while (t < 6000u + hold_ms) {
        t += 10u;
        *ticks += 20;
        sstep(ctl, t, *ticks, *ticks);
    }
    set_sensors(0u);
    t += 10u;
    *ticks += 20;
    sstep(ctl, t, *ticks, *ticks);
    return t;
}

static uint32_t scorrect(MOTION_CONTROLLER *ctl, uint32_t hold_ms, int32_t *ticks)
{
    return scorrect_on(ctl, 1u, hold_ms, ticks);  /* R black: steer right */
}

/* #23: lost-line detection in the straddle model (S2_LINE_LOST_MM, then a
 * probe of S2_LINE_PROBE_MM before stopping). */
static void straddle_lost_tests(void)
{
    MOTION_CONTROLLER ctl;
    int32_t ticks, t0;
    uint32_t t;
    const float lost = (float)S2_LINE_LOST_MM, stop = lost + (float)S2_LINE_PROBE_MM;

    /* A hard right correction that clears to white and never sees black
     * again: plain following up to S2_LINE_LOST_MM, then the probe steers
     * right (soft) toward where the tape was, then a recoverable stop. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS, &ticks);
    EXPECT(g_b_lost_watch && g_lost_side == 1u && !g_b_halted);
    t0 = ticks;
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(lost - 10.0f), 20);
    EXPECT(!g_b_halted && g_b_lost_watch && !g_b_lost_probe && ctl.target_value == 0.0f);
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(lost + 5.0f), 20);
    EXPECT(!g_b_halted && g_b_lost_probe && ctl.target_value == S2_LINE_PROBE_STEER);
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(stop - 10.0f), 20);
    EXPECT(!g_b_halted && g_b_lost_probe && ctl.target_value == S2_LINE_PROBE_STEER);
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(stop + 5.0f), 20);
    EXPECT(g_b_halted && g_b_line_halt && g_b_lost_stop && !g_b_lost_seen_black && !enabled &&
           !g_b_lost_probe);

    /* Held still on white: both white proves nothing, so no re-arm. */
    for (uint32_t i = 0; i < 300u; ++i) { t += 10u; sstep(&ctl, t, ticks, ticks); }
    EXPECT(g_b_halted && g_b_lost_stop);
    /* The tape under R once, then still: re-arms S2_LINE_REARM_MS later,
     * with no correction left over from before the stop. */
    set_sensors(1u);
    t += 10u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(g_b_lost_seen_black && g_b_halted);
    set_sensors(0u);
    uint32_t seen = t;
    while (t < seen + S2_LINE_REARM_MS - 20u) { t += 10u; sstep(&ctl, t, ticks, ticks); }
    EXPECT(g_b_halted);
    t += 20u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(!g_b_halted && enabled && !g_b_lost_stop && g_steer_action == 0u && !g_b_lost_watch);

    /* The 23 Nov false stop: tape still between C and R. The probe steers
     * toward it, the other sensor (C) sees the tape, and following carries
     * on with a normal left correction: no stop. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS, &ticks);
    t0 = ticks;
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(lost + 30.0f), 20);
    EXPECT(g_b_lost_probe && !g_b_halted);
    set_sensors(2u);
    t += 10u;
    ticks += 20;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(!g_b_halted && !g_b_lost_watch && !g_b_lost_probe && ctl.target_value < 0.0f);
    EXPECT(g_lost_max_mm >= (uint32_t)lost + 25u);       /* shows in `watch max` */
    set_sensors(0u);
    t = sdrive(&ctl, t, &ticks, ticks + ticks_for_mm(2.0f * stop), 20);
    EXPECT(!g_b_halted);                 /* that C correction was short: no watch */

    /* A left correction (C black) probes left. */
    t = scorrect_on(&ctl, 2u, S2_LINE_LOST_ARM_MS, &ticks);
    EXPECT(g_b_lost_watch && g_lost_side == 2u);
    t = sdrive(&ctl, t, &ticks, ticks + ticks_for_mm(lost + 5.0f), 20);
    EXPECT(g_b_lost_probe && ctl.target_value == -S2_LINE_PROBE_STEER);

    /* PAUSE + RESUME after a lost stop re-arms without a black reading. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS, &ticks);
    t = sdrive(&ctl, t, &ticks, ticks + ticks_for_mm(stop + 5.0f), 20);
    EXPECT(g_b_halted && g_b_lost_stop);
    send_command(S1_COMMAND_PAUSE);
    t += 10u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(g_b_paused);
    send_command(S1_COMMAND_RESUME);
    t += 10u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(!g_b_paused && g_b_lost_seen_black && g_b_halted);
    uint32_t resumed = t;
    while (t < resumed + S2_LINE_REARM_MS + 10u) { t += 10u; sstep(&ctl, t, ticks, ticks); }
    EXPECT(!g_b_halted && enabled && !g_b_lost_stop);

    /* PAUSE in the middle of a correction, RESUME: the old correction (and
     * its hard timer) does not carry over. */
    sfollow(&ctl);
    set_sensors(1u);
    t = 6000u;
    for (uint32_t i = 0; i <= S2_LINE_HARD_MS / 10u; ++i) { sstep(&ctl, t, 1, 1); t += 10u; }
    EXPECT(g_steer_action == 1u && ctl.target_value == 1.0f);     /* hard */
    send_command(S1_COMMAND_PAUSE);
    sstep(&ctl, t, 1, 1);
    EXPECT(g_b_paused);
    for (uint32_t i = 0; i < 200u; ++i) {           /* 2 s paused, motors stopping */
        t += 10u;
        motion_control_step(&ctl, t, 1, 1);
        sstep(&ctl, t, 1, 1);
    }
    EXPECT(g_b_paused && !g_b_halted && g_steer_action == 1u);  /* kept while paused */
    t += 10u;
    send_command(S1_COMMAND_RESUME);
    sstep(&ctl, t, 1, 1);
    EXPECT(!g_b_paused && g_steer_action == 0u && !g_b_lost_watch);
    for (uint32_t i = 0; i < 4u; ++i) {          /* past the 30 ms settle */
        t += 10u;
        motion_control_step(&ctl, t, 1, 1);
        sstep(&ctl, t, 1, 1);
    }
    EXPECT(g_steer_action == 1u && ctl.target_value == 0.5f);     /* soft again */

    /* Black again within S2_LINE_LOST_MM: the watch clears, no probe, and
     * the distance is kept for the `steer ... watch max=` log. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS, &ticks);
    t0 = ticks;
    t = sdrive(&ctl, t, &ticks, t0 + ticks_for_mm(100.0f), 20);
    EXPECT(!g_b_lost_probe);
    set_sensors(2u);
    t += 10u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(!g_b_halted && !g_b_lost_watch && g_lost_max_mm >= 95u && g_lost_max_mm <= 115u);

    /* A correction shorter than S2_LINE_LOST_ARM_MS never starts a watch. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS - 20u, &ticks);
    EXPECT(!g_b_lost_watch);
    t = sdrive(&ctl, t, &ticks, ticks + ticks_for_mm(3.0f * stop), 20);
    EXPECT(!g_b_halted && ctl.target_value == 0.0f);

    /* A gap in line following (turn, avoidance, pause) drops the watch and
     * any probe, so ticks counted elsewhere cannot trip it. */
    t = scorrect(&ctl, S2_LINE_LOST_ARM_MS, &ticks);
    t = sdrive(&ctl, t, &ticks, ticks + ticks_for_mm(lost + 5.0f), 20);
    EXPECT(g_b_lost_probe);
    ticks += ticks_for_mm(3.0f * stop);
    t += S2_LINE_LOST_GAP_MS + 10u;
    sstep(&ctl, t, ticks, ticks);
    EXPECT(!g_b_halted && !g_b_lost_watch && !g_b_lost_probe);

    /* A long straight with no correction never stops or probes (the 2 s
     * deadline's failure, 21 Nov). */
    sfollow(&ctl);
    ticks = 0;
    t = sdrive(&ctl, 6000u, &ticks, ticks_for_mm(20.0f * stop), 20);
    EXPECT(!g_b_halted && !g_b_lost_watch && ctl.target_value == 0.0f);
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
        EXPECT(!g_b_halted && ctl.target_value == 0.0f);
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
    EXPECT(!g_b_turning && g_b_leaving_junction && ctl.target_value == 0.0f &&
           ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    /* Still on that crossing: a mapped letter read now must not turn here. */
    send_map("FAZW");
    sstep(&ctl, 6010, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'A' - 'A';
    has_event = true;
    sstep(&ctl, 6020, 1, 1);
    EXPECT(!g_b_turning && (g_pending_letter & 0x80u));        /* kept for the next one */
    set_sensors(0u);
    sstep(&ctl, 6100, 1, 1);
    EXPECT(g_b_leaving_junction);
    sstep(&ctl, 6000 + S2_JUNCTION_SETTLE_MS, 1, 1);
    EXPECT(!g_b_leaving_junction);
    set_sensors(3u);                                       /* the next crossing */
    sstep(&ctl, 6000 + S2_JUNCTION_SETTLE_MS + 10u, 1, 1);
    EXPECT(g_b_turning && g_turn_cmd == LINE_BARCODE_CMD_RIGHT);

    /* Mapped letter: drive onto the crossing, pivot, then follow again. */
    sfollow(&ctl);
    send_map("FAZW");                       /* slots L,R,S,U: A = right turn */
    sstep(&ctl, 6000, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'A' - 'A';
    has_event = true;
    sstep(&ctl, 6010, 1, 1);
    EXPECT((g_pending_letter & 0x80u) && g_last_letter == 1u);
    set_sensors(3u);
    sstep(&ctl, 6020, 1, 1);
    EXPECT(g_b_turning && g_turn_stage == TURN_ADVANCE && !enabled &&
           ctl.status.active_command == MOTION_CMD_FORWARD_CM);
    motion_control_step(&ctl, 6030, 1, 1);
    motion_control_step(&ctl, 6040, 1 + 60, 1 + 60);      /* > 5 cm at 10 ticks/cm */
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    sstep(&ctl, 6050, 61, 61);
    EXPECT(g_b_turning && g_turn_stage == TURN_PIVOT &&
           ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);
    motion_control_step(&ctl, 6060, 61, 61);
    motion_control_step(&ctl, 6070, 61 + 200, 61);        /* left wheel: 90 deg x 2 */
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    sstep(&ctl, 6080, 261, 61);
    EXPECT(!g_b_turning && enabled && g_b_leaving_junction && !g_b_halted);

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
    EXPECT(!g_b_turning && g_b_leaving_junction && !(g_pending_letter & 0x80u));

    /* S3-U33 (VC part, #23): an invalid frame stores no letter, so the next
     * junction goes straight on with no turn. */
    sfollow(&ctl);
    send_map("FAZW");
    sstep(&ctl, 6000, 1, 1);
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_INVALID << 3u);
    has_event = true;
    sstep(&ctl, 6010, 1, 1);
    EXPECT(!g_b_halted && !(g_pending_letter & 0x80u));
    set_sensors(3u);
    sstep(&ctl, 6020, 1, 1);
    EXPECT(!g_b_turning && g_b_leaving_junction && !g_b_halted &&
           ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    /* An S3 fault stops at once and re-arms on a healthy snapshot held
     * still (both white is fine in this model). */
    sfollow(&ctl);
    snapshot.data[1] = LINE_BARCODE_FAULT;
    sstep(&ctl, 6000, 1, 1);
    EXPECT(g_b_halted && g_b_line_halt && !enabled);
    set_sensors(0u);
    sstep(&ctl, 6100, 1, 1);
    sstep(&ctl, 6100 + S2_LINE_REARM_MS - 101u, 1, 1);
    EXPECT(g_b_halted);
    sstep(&ctl, 6000 + S2_LINE_REARM_MS, 1, 1);
    EXPECT(!g_b_halted && enabled);

    /* RESUME after PAUSE works on both-white in this model. */
    sfollow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    sstep(&ctl, 6000, 1, 1);
    EXPECT(g_b_paused);
    send_command(S1_COMMAND_RESUME);
    sstep(&ctl, 6010, 1, 1);
    EXPECT(!g_b_paused && enabled);
}

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-3f)

/* Step the controller every 10 ms, n times, with fixed encoder counts. */
static uint32_t msteps(MOTION_CONTROLLER *m, uint32_t now, int n, int32_t l, int32_t r)
{
    for (int i = 0; i < n; ++i) { now += 10u; motion_control_step(m, now, l, r); }
    return now;
}

/* #23: the straight trim's I term (S2_FOLLOW_STRAIGHT_KI) learns a steady
 * motor mismatch and keeps it across stretches. */
static void follow_bias_tests(void)
{
    MOTION_CONTROLLER m;
    MOTION_CONFIG cfg = alltest_config();
    MOTION_COMMAND follow = {MOTION_CMD_FOLLOW_LINE, 0.0f};
    const float L = S2_FOLLOW_LEFT_PWM_PERCENT, R = S2_FOLLOW_RIGHT_PWM_PERCENT;
    const float lt = S2_ENC_LEFT_TICKS_PER_CM, rt = S2_ENC_RIGHT_TICKS_PER_CM;
    const float cap = S2_FOLLOW_STRAIGHT_MAX_TRIM + S2_FOLLOW_STRAIGHT_MAX_BIAS;
    const int32_t r11 = (int32_t)(1.1f * rt), r5 = (int32_t)(5.0f * rt);
    float bias;
    uint32_t now;

    cfg.follow_straight_ki = S2_FOLLOW_STRAIGHT_KI;
    cfg.follow_straight_max_bias = S2_FOLLOW_STRAIGHT_MAX_BIAS;
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    EXPECT(motion_control_command(&m, &follow, 0u, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, 0u, 10, 0, 0);
    EXPECT(m.straight_bias == 0.0f);                       /* level: nothing learnt */

    /* 1 s with the right wheel 1.1 cm ahead: KI x (1.1 - deadband) x 1 s. */
    now = msteps(&m, now, 100, 0, r11);
    EXPECT(fabsf(m.straight_bias - S2_FOLLOW_STRAIGHT_KI *
                 ((float)r11 / rt - S2_FOLLOW_STRAIGHT_DEADBAND_CM)) < 0.02f);
    bias = m.straight_bias;

    /* A correction ends the stretch: the P trim restarts at 0, the bias is
     * kept and still applied, so the next stretch starts corrected. */
    follow.value = 0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, r11) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, r11);
    EXPECT(m.straight_bias == bias);                       /* not learnt while steering */
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, r11) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, r11);
    EXPECT(m.straight_trim == 0.0f && m.straight_bias == bias);
    EXPECT(NEAR(m.status.left_pwm_percent, L + bias) && NEAR(m.status.right_pwm_percent, R));

    /* A large steady difference: capped at MAX_BIAS, PWM at L + trim + bias. */
    now = msteps(&m, now, 1000, 0, r5);
    EXPECT(NEAR(m.straight_bias, S2_FOLLOW_STRAIGHT_MAX_BIAS));
    EXPECT(NEAR(m.status.left_pwm_percent, L + cap) && NEAR(m.status.right_pwm_percent, R));

    /* The left wheel ahead instead: the bias swings negative and boosts
     * the right wheel; the left is never cut. */
    follow.value = 0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, r5) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, r5);
    follow.value = 0.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, r5) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, r5);
    now = msteps(&m, now, 300, (int32_t)(10.0f * lt), r5);
    EXPECT(NEAR(m.straight_bias, -S2_FOLLOW_STRAIGHT_MAX_BIAS));
    EXPECT(NEAR(m.status.left_pwm_percent, L) && NEAR(m.status.right_pwm_percent, R + cap));

    /* Only init clears it. */
    EXPECT(motion_control_init(&m, &cfg, now, 0, 0));
    EXPECT(m.straight_bias == 0.0f);

    /* Config: negative KI or bias, or cruise + trim + bias over the PWM
     * ceiling, is rejected. */
    cfg.follow_straight_ki = -1.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg.follow_straight_ki = S2_FOLLOW_STRAIGHT_KI;
    cfg.follow_straight_max_bias = -1.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg.follow_straight_max_bias = cfg.maximum_pwm_percent - R;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
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

    /* #23: a right correction while scanning gives the scanner-side (left)
     * wheel no outer boost, so GP28 does not speed up over the bars; the
     * right wheel still slows (soft, then hard). Not scanning: boost as
     * before. */
    motion_control_set_scanning(&m, true);
    follow.value = 0.5f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 100, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, L * S2_FOLLOW_SCAN_SCALE));
    EXPECT(NEAR(m.status.right_pwm_percent,
                S2_FOLLOW_INNER_PWM_PERCENT * S2_FOLLOW_SCAN_SCALE));
    follow.value = 1.0f;
    EXPECT(motion_control_command(&m, &follow, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 100, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, L * S2_FOLLOW_SCAN_SCALE));
    EXPECT(NEAR(m.status.right_pwm_percent,
                S2_FOLLOW_HARD_INNER_PWM_PERCENT * S2_FOLLOW_SCAN_SCALE));
    motion_control_set_scanning(&m, false);
    now = msteps(&m, now, 100, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, L + S2_FOLLOW_BOOST_PWM_PERCENT));
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
    /* The advance keeps the follow PWM (no ramp-down to the move PWM). */
    now = msteps(&m, now, 1, 90, 90);                       /* 0.76 cm */
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD &&
           NEAR(m.status.left_pwm_percent, L) &&
           NEAR(m.status.right_pwm_percent, R));
    now = msteps(&m, now, 1, 95, 95);                       /* 0.80 cm */
    EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);

    /* The pivot starts straight after from the advance's PWM (carry). Right
     * pivot: right wheel stops at once, left at the junction pivot PWM,
     * S2 turn calibration and no 2-degree tolerance. */
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90.0f};
    EXPECT(motion_control_command(&m, &cmd, now + 1u, 95, 95) == MOTION_RESULT_OK);
    /* Carry: the advance held the follow PWM. */
    EXPECT(NEAR(m.status.left_pwm_percent, L));
    {
        float carried = m.status.left_pwm_percent;
        float pivot = S2_JUNCTION_RIGHT_PIVOT_PWM_PERCENT;
        /* One step later: the left wheel ramps from the carried PWM toward
         * the pivot PWM, at the move ramp, and never past it. */
        now = msteps(&m, now + 1u, 1, 96, 95);
        EXPECT(m.status.right_pwm_percent == 0.0f &&
               NEAR(m.status.left_pwm_percent,
                    carried < pivot ? fminf(pivot, carried + move_step) :
                                      fmaxf(pivot, carried - move_step)));
        for (int i = 0; i < 40; ++i) now = msteps(&m, now, 1, 96, 95);
        EXPECT(NEAR(m.status.left_pwm_percent, pivot));
    }
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
        float tpd = S2_AVOID_LEFT_TURN_TICKS_PER_DEGREE;  /* measured calibration (23 Nov) */
        now = msteps(&m, now, 1, (int32_t)(90.0f * tpd) - 5, 0);
        EXPECT(m.status.state == MOTION_STATE_TURNING_RIGHT);   /* S2 calibration would be done */
        now = msteps(&m, now, 1, (int32_t)(90.0f * tpd) + 1, 0);
        EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete);
    }

    /* S5 spin (23 Nov): both wheels counter-rotate at the spin PWM; each
     * stops on its own after half its pivot arc; the turn ends when both
     * have; FORWARD is refused; a spin never carries its reverse PWM on. */
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_STOP, 0}, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, 0, 0);
    motion_control_set_profile(&m, MOTION_PROFILE_SPIN);
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_FORWARD_CM, 10.0f}, now, 0, 0) ==
           MOTION_RESULT_INVALID_COMMAND);
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_LEFT_DEG, 90.0f};
    EXPECT(motion_control_command(&m, &cmd, now, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, now, 1, -1, 1);
    EXPECT(m.status.left_pwm_percent < 0.0f && m.status.right_pwm_percent > 0.0f);  /* ramps from rest */
    now = msteps(&m, now, 30, -50, 50);
    EXPECT(m.status.state == MOTION_STATE_TURNING_LEFT &&
           NEAR(m.status.left_pwm_percent, -S2_AVOID_SPIN_PWM_PERCENT) &&
           NEAR(m.status.right_pwm_percent, S2_AVOID_SPIN_PWM_PERCENT));
    {
        int32_t l90 = (int32_t)(45.0f * S2_AVOID_LEFT_TURN_TICKS_PER_DEGREE) + 1;
        int32_t r90 = (int32_t)(45.0f * S2_AVOID_RIGHT_TURN_TICKS_PER_DEGREE) + 1;
        now = msteps(&m, now, 1, -(l90 - 3), r90 - 3);
        EXPECT(m.status.state == MOTION_STATE_TURNING_LEFT);      /* half arc: not there yet */
        now = msteps(&m, now, 1, -l90, r90 - 20);
        EXPECT(m.status.state == MOTION_STATE_TURNING_LEFT &&     /* left done: it stops */
               m.status.left_pwm_percent == 0.0f &&
               NEAR(m.status.right_pwm_percent, S2_AVOID_SPIN_PWM_PERCENT));
        now = msteps(&m, now, 1, -l90, r90);
        EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete &&
               m.status.left_pwm_percent == 0.0f && m.status.right_pwm_percent == 0.0f &&
               !m.carry_valid);
        /* Right spin: the mirror; a reversing wheel that stalls faults. */
        motion_control_set_profile(&m, MOTION_PROFILE_SPIN);
        cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90.0f};
        EXPECT(motion_control_command(&m, &cmd, now, -l90, r90) == MOTION_RESULT_OK);
        now = msteps(&m, now, 1, -l90 + 1, r90 - 1);
        now = msteps(&m, now, 30, -l90 + 50, r90 - 50);
        EXPECT(m.status.state == MOTION_STATE_TURNING_RIGHT &&
               NEAR(m.status.left_pwm_percent, S2_AVOID_SPIN_PWM_PERCENT) &&
               NEAR(m.status.right_pwm_percent, -S2_AVOID_SPIN_PWM_PERCENT));
        now = msteps(&m, now, 60, -l90 + 50, r90 - 50);
        EXPECT(m.status.state == MOTION_STATE_FAULT);
    }
    /* S5 reverse (23 Nov): FORWARD_CM backwards at the leg PWM, heading held
     * on distance magnitudes (the wheel that has gone further is slowed),
     * ends on the distance, no carry; turns are refused. */
    cfg = alltest_config();
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    motion_control_set_profile(&m, MOTION_PROFILE_REVERSE);
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_TURN_LEFT_DEG, 90.0f}, 0u, 0, 0) ==
           MOTION_RESULT_INVALID_COMMAND);
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM, 15.0f};
    EXPECT(motion_control_command(&m, &cmd, 0u, 0, 0) == MOTION_RESULT_OK);
    now = msteps(&m, 0u, 1, 0, 0);
    EXPECT(NEAR(m.status.left_pwm_percent, -move_step) && NEAR(m.status.right_pwm_percent, -move_step));
    now = msteps(&m, now, 100, 0, 0);
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD &&
           NEAR(m.status.left_pwm_percent, -S2_MOVE_LEFT_PWM_PERCENT) &&
           NEAR(m.status.right_pwm_percent, -S2_MOVE_RIGHT_PWM_PERCENT));
    now = msteps(&m, now, 100, -(int32_t)(2.0f * lt), -(int32_t)(2.5f * rt));   /* right 0.5 cm ahead */
    EXPECT(fabsf(m.status.right_pwm_percent +
                 (S2_MOVE_RIGHT_PWM_PERCENT - S2_AVOID_HEADING_KP * 0.5f)) < 0.05f &&
           NEAR(m.status.left_pwm_percent, -S2_MOVE_LEFT_PWM_PERCENT));
    now = msteps(&m, now, 1, -(int32_t)(14.95f * lt), -(int32_t)(14.95f * rt));
    EXPECT(m.status.state == MOTION_STATE_MOVING_FORWARD);
    now = msteps(&m, now, 1, -(int32_t)(15.01f * lt), -(int32_t)(15.01f * rt));
    EXPECT(m.status.state == MOTION_STATE_IDLE && m.status.motion_complete && !m.carry_valid);

    /* No spin PWM configured: spins are refused. */
    cfg = alltest_config();
    cfg.avoid_spin_pwm_percent = 0.0f;
    EXPECT(motion_control_init(&m, &cfg, 0u, 0, 0));
    motion_control_set_profile(&m, MOTION_PROFILE_SPIN);
    EXPECT(motion_control_command(&m, &(MOTION_COMMAND){MOTION_CMD_TURN_LEFT_DEG, 90.0f}, 0u, 0, 0) ==
           MOTION_RESULT_INVALID_COMMAND);
    cfg = alltest_config();
    cfg.avoid_spin_pwm_percent = 101.0f;
    EXPECT(!motion_control_init(&m, &cfg, 0u, 0, 0));
    cfg = alltest_config();

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
    /* Right-wheel ticks that cover the longer junction advance (6.3 cm at
     * about 118 ticks/cm is 745), with room to spare. */
    const int32_t adv = 800;

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
        EXPECT(g_b_turning && g_turn_stage == TURN_ADVANCE && g_turn_cmd == LINE_BARCODE_CMD_UTURN);
        EXPECT(ctl.status.active_command == MOTION_CMD_FORWARD_CM &&
               ctl.status.left_pwm_percent > 10.0f);        /* not stopped first */
        now = msteps(&ctl, now + 1u, 1, adv, adv);          /* > the advance */
        EXPECT(ctl.status.state == MOTION_STATE_IDLE);
        sstep(&ctl, now + 1u, adv, adv);
        EXPECT(g_turn_stage == TURN_PIVOT && ctl.active_profile == MOTION_PROFILE_UTURN &&
               ctl.status.left_pwm_percent > 10.0f);        /* carried on */
        now = msteps(&ctl, now + 1u, 30, adv - 300, adv + 300);
        EXPECT(ctl.status.left_pwm_percent < 0.0f);         /* reversing */
        set_sensors(centre_late ? 0u : 2u);                 /* centre black early */
        sstep(&ctl, now, adv - half + 100, adv + half - 100);
        EXPECT(g_b_turning);                                    /* arc not done yet */
        set_sensors(0u);
        sstep(&ctl, now + 1u, adv - half, adv + half + 1);
        EXPECT(g_b_turning);                                    /* arc done, centre white */
        set_sensors(2u);
        sstep(&ctl, now + 2u, adv - half, adv + half + 2);
        EXPECT(!g_b_turning && !g_b_halted && enabled && g_b_leaving_junction);
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
    motion_control_step(&ctl, 5060, adv, adv);
    sstep(&ctl, 5061, adv, adv);
    EXPECT(g_turn_stage == TURN_PIVOT && ctl.active_profile == MOTION_PROFILE_JUNCTION &&
           ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);

    /* A left or right pivot stops on the line: past 90 % of its arc, the
     * centre sensor on black ends it early; before 90 % or with no line it
     * runs the full tick count. */
    for (int side = 0; side < 2; ++side) {                  /* 0 right, 1 left */
        const char letter = side ? 'F' : 'A';
        const float tpd = side ? S2_RIGHT_TURN_TICKS_PER_DEGREE :
                                 S2_LEFT_TURN_TICKS_PER_DEGREE;
        const int32_t arc = (int32_t)(90.0f * tpd);
        for (int variant = 0; variant < 4; ++variant) {     /* 0 late+black, 1 early, 2 late+white, 3 mid */
            /* mid is 0.8 of the arc: a right pivot (min 0.9) keeps turning, a
             * left pivot (min 0.8) may stop on the line from there. */
            int32_t at = (int32_t)(arc * (variant == 1 ? 0.4f : variant == 3 ? 0.85f : 0.95f));
            sfollow(&ctl);
            send_map("FAZW");
            queued = snapshot;
            queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
            queued.data[2] = (uint8_t)(letter - 'A');
            has_event = true;
            sstep(&ctl, 5040, 0, 0);
            set_sensors(3u);
            sstep(&ctl, 5050, 0, 0);
            motion_control_step(&ctl, 5060, adv, adv);
            sstep(&ctl, 5061, adv, adv);
            EXPECT(g_turn_stage == TURN_PIVOT);
            set_sensors(variant == 2 ? 0u : 2u);
            now = msteps(&ctl, 5062, 3, side ? adv : adv + at, side ? adv + at : adv);
            sstep(&ctl, now, side ? adv : adv + at, side ? adv + at : adv);
            if (variant == 0 || (variant == 3 && side == 1)) {
                EXPECT(!g_b_turning && !g_b_halted && enabled);      /* stopped on the line */
            } else {
                EXPECT(g_b_turning && g_turn_stage == TURN_PIVOT); /* kept turning */
            }
        }
    }

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
        /* A turn with the spin bit runs as a spin; without it, a pivot. */
        req.move = S5_MOVE_TURN_R;
        req.value = 90u;
        req.b_spin = TRUE;
        vc_stop(&c);
        motion_control_step(&ctl, 6001u, 0, 0);
        EXPECT(vc_start_move(&c, &req));
        EXPECT(ctl.active_profile == MOTION_PROFILE_SPIN &&
               ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);
        req.b_spin = FALSE;
        vc_stop(&c);
        motion_control_step(&ctl, 6002u, 0, 0);
        EXPECT(vc_start_move(&c, &req));
        EXPECT(ctl.active_profile == MOTION_PROFILE_AVOID &&
               ctl.status.active_command == MOTION_CMD_TURN_RIGHT_DEG);
        /* A leg with the reverse bit drives backwards. */
        req.move = S5_MOVE_FWD;
        req.value = 15u;
        req.b_reverse = TRUE;
        vc_stop(&c);
        motion_control_step(&ctl, 6003u, 0, 0);
        EXPECT(vc_start_move(&c, &req));
        EXPECT(ctl.active_profile == MOTION_PROFILE_REVERSE &&
               ctl.status.active_command == MOTION_CMD_FORWARD_CM);
        req.b_reverse = FALSE;
        g_steer_action = 1u;
        EXPECT(vc_give_back(&c));
        EXPECT(g_b_leaving_junction && g_junction_done == 6000u && g_steer_action == 0u);
    }

    all_numbers = false;
}

#if S5_REAL_EXECUTOR
/* ---- INT-13: VC <-> S5 hand-over with the real executor (#20) ----------- */

static unsigned max_pwm_seen;

/* What task_motion does each period: the VC step, then the motion step. */
static void vstep(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    unsigned pwm;
    fake_imu.sample_count++;
    robot_control_step(ctl, now, l, r);
    motion_control_step(ctl, now, l, r);
    pwm = (unsigned)(ctl->status.left_pwm_percent + ctl->status.right_pwm_percent);
    if (pwm > max_pwm_seen) max_pwm_seen = pwm;
}

static void s5_reset(void)
{
    s5_flags = 0u; s5_seq = 0u; s5_word = 0u; /* NONE */
    max_pwm_seen = 0u;
}

/* What S5's s5_post_and_wait() does before posting: forget old replies. */
static void s5_forget(UINT wanted) { s5_flags &= ~wanted; }

static bool motors_off(const MOTION_CONTROLLER *ctl)
{
    return ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0;
}

/* Armed and following, with the real executor reporting FOLLOWING. */
static void s5_follow(MOTION_CONTROLLER *ctl)
{
    reset(ctl);
    s5_reset();
    vstep(ctl, 5000, 0, 0);
    vstep(ctl, 5030, 1, 1);
    EXPECT(g_b_armed && enabled && !g_b_halted);
    EXPECT(ctl->status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT((s5_flags & S5_EVT_FOLLOWING) != 0u);
}

/* S5 asks for HOLD at 'now'; step until the VC replies STOPPED. */
static uint32_t s5_hold(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    uint32_t t;
    s5_forget(S5_EVT_STOPPED);
    s5_post(S5_REQ_HOLD, S5_MOVE_FWD, 0u, FALSE);
    vstep(ctl, now, l, r);
    EXPECT(g_b_in_avoid && !enabled);                   /* S5 has control */
    EXPECT((s5_flags & S5_EVT_FOLLOWING) == 0u);
    EXPECT(ctl->status.active_command != MOTION_CMD_FOLLOW_LINE);
    EXPECT((s5_flags & S5_EVT_STOPPED) == 0u);      /* not before the settle */
    for (t = now + 10u; t <= now + 1000u && (s5_flags & S5_EVT_STOPPED) == 0u; t += 10u)
        vstep(ctl, t, l, r);
    EXPECT((s5_flags & S5_EVT_STOPPED) != 0u);
    EXPECT(t - now >= S2_AVOID_HOLD_SETTLE_MS);     /* coasted first */
    EXPECT(motors_off(ctl) && !g_b_halted);
    return t;
}

static void s5_handover_tests(void)
{
    MOTION_CONTROLLER ctl;
    uint32_t t;
    int32_t ticks;

    /* (a) Start-up HOLD (servo swing): the VC does not arm until NONE. */
    reset(&ctl);
    s5_reset();
    s5_post(S5_REQ_HOLD, S5_MOVE_FWD, 0u, FALSE);
    for (t = 5000u; t <= 6000u; t += 10u) vstep(&ctl, t, 0, 0);
    EXPECT(!g_b_armed && !g_b_halted && max_pwm_seen == 0u);
    s5_post(S5_REQ_NONE, S5_MOVE_FWD, 0u, FALSE);
    vstep(&ctl, 6010, 0, 0);
    vstep(&ctl, 6040, 1, 1);
    EXPECT(g_b_armed && enabled && (s5_flags & S5_EVT_FOLLOWING) != 0u);

    /* (b) Full hand-over: HOLD -> STOPPED, MOVE -> MOTION_DONE,
     *     stop-on-line MOVE -> LINE_FOUND, RESUME -> line following. */
    s5_follow(&ctl);
    t = s5_hold(&ctl, 6000u, 1, 1);

    s5_forget(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED);
    s5_post(S5_REQ_MOVE, S5_MOVE_FWD, 10u, FALSE);
    vstep(&ctl, t += 10u, 1, 1);
    EXPECT(g_b_in_avoid && ctl.status.active_command == MOTION_CMD_FORWARD_CM &&
           ctl.next_profile == MOTION_PROFILE_AVOID);  /* applied with all_test numbers */
    for (ticks = 1; ticks < 400 && (s5_flags & S5_EVT_MOTION_DONE) == 0u; ticks += 2)
        vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT((s5_flags & S5_EVT_MOTION_DONE) != 0u);
    EXPECT((s5_flags & S5_EVT_MOVE_FAILED) == 0u);
    EXPECT(ticks >= 100);                           /* 10 cm x 10 ticks/cm */
    EXPECT(motors_off(&ctl) && !g_b_halted && !enabled);

    snapshot.data[0] = 0u;                          /* both sensors on white */
    s5_forget(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED | S5_EVT_LINE_FOUND);
    s5_post(S5_REQ_MOVE, S5_MOVE_FWD, 50u, TRUE);
    vstep(&ctl, t += 10u, ticks, ticks);
    for (int i = 0; i < 5; ++i) { ticks += 2; vstep(&ctl, t += 10u, ticks, ticks); }
    EXPECT((s5_flags & (S5_EVT_LINE_FOUND | S5_EVT_MOTION_DONE)) == 0u);
    snapshot.data[0] = 2u;                          /* C sees the line */
    ticks += 2;
    vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT((s5_flags & S5_EVT_LINE_FOUND) != 0u);
    EXPECT(motors_off(&ctl) && !g_b_halted);

    snapshot = (line_barcode_msg_t){{3u | (2u << 3u), LINE_BARCODE_FOLLOW, 0u}};
    s5_post(S5_REQ_RESUME, S5_MOVE_FWD, 0u, FALSE);
    vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT(enabled && !g_b_halted);                     /* line following back */
    vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT(!g_b_in_avoid && motors_off(&ctl));          /* VC's 30 ms grace */
    vstep(&ctl, t += 30u, ticks, ticks);
    EXPECT(!g_b_in_avoid && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT((s5_flags & S5_EVT_FOLLOWING) != 0u);

    /* (c1) Wheels stalled during an S5 move: the motion controller's
     *      encoder-stall fault makes the VC latch its own safety stop (S5
     *      gets no reply; its 25 s wait then posts SAFE_STOP). */
    s5_follow(&ctl);
    t = s5_hold(&ctl, 6000u, 1, 1);
    s5_forget(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED);
    s5_post(S5_REQ_MOVE, S5_MOVE_FWD, 30u, FALSE);
    {
        uint32_t t0 = t;
        while (!g_b_halted && t < t0 + 5000u) vstep(&ctl, t += 10u, 1, 1);
        EXPECT(g_b_halted && !g_b_line_halt && motors_off(&ctl));
        EXPECT(t - t0 <= 2000u);                    /* stall caught quickly */
        EXPECT((s5_flags & (S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED)) == 0u);
    }
    s5_post(S5_REQ_SAFE_STOP, S5_MOVE_FWD, 0u, FALSE);
    for (int i = 0; i < 100; ++i) vstep(&ctl, t += 10u, 1, 1);
    EXPECT(g_b_halted && motors_off(&ctl));

    /* (c2) A move that keeps going past S2_AVOID_MOVE_TIMEOUT_MS fails with
     *      S5_EVT_MOVE_FAILED and the motors stop; S5's SAFE_STOP then
     *      latches until reboot, even if the word changes again. */
    s5_follow(&ctl);
    t = s5_hold(&ctl, 6000u, 1, 1);
    s5_forget(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED);
    s5_post(S5_REQ_MOVE, S5_MOVE_FWD, S5_REQ_VALUE_MAX, FALSE); /* 1023 cm */
    {
        uint32_t t0 = t;
        ticks = 1;
        while ((s5_flags & S5_EVT_MOVE_FAILED) == 0u && !g_b_halted &&
               t < t0 + S2_AVOID_MOVE_TIMEOUT_MS + 1000u) {
            ++ticks;                                /* creeping: no stall */
            vstep(&ctl, t += 10u, ticks, ticks);
        }
        EXPECT((s5_flags & S5_EVT_MOVE_FAILED) != 0u && !g_b_halted);
        EXPECT(t - t0 > S2_AVOID_MOVE_TIMEOUT_MS);
        EXPECT((s5_flags & S5_EVT_MOTION_DONE) == 0u && motors_off(&ctl));
    }
    s5_post(S5_REQ_SAFE_STOP, S5_MOVE_FWD, 0u, FALSE);
    vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT(g_b_halted && !g_b_line_halt && motors_off(&ctl) && !enabled);
    s5_post(S5_REQ_NONE, S5_MOVE_FWD, 0u, FALSE);   /* too late: latched */
    for (int i = 0; i < 300; ++i) vstep(&ctl, t += 10u, ticks, ticks);
    EXPECT(g_b_halted && motors_off(&ctl));

    /* (c3) A move the controller rejects (0 cm) fails at once. */
    s5_follow(&ctl);
    t = s5_hold(&ctl, 6000u, 1, 1);
    s5_forget(S5_EVT_MOTION_DONE | S5_EVT_MOVE_FAILED);
    s5_post(S5_REQ_MOVE, S5_MOVE_FWD, 0u, FALSE);
    vstep(&ctl, t += 10u, 1, 1);
    EXPECT((s5_flags & S5_EVT_MOVE_FAILED) != 0u && motors_off(&ctl));

    /* (d) A corrupt S5 word while following: safe stop at once, latched. */
    s5_follow(&ctl);
    s5_word = 5u;                                   /* reserved kind 5 */
    vstep(&ctl, 6000, 2, 2);
    EXPECT(g_b_halted && !g_b_line_halt && motors_off(&ctl));
    EXPECT((s5_flags & S5_EVT_FOLLOWING) == 0u);
    s5_word = 0u;
    for (int i = 0; i < 300; ++i) vstep(&ctl, 6010u + 10u * (unsigned)i, 2, 2);
    EXPECT(g_b_halted && motors_off(&ctl));

    /* (e) #47 CAL_FAIL: S5 asks for SAFE_STOP before the VC has armed.
     *     The wheels must never be driven. */
    reset(&ctl);
    s5_reset();
    s5_post(S5_REQ_SAFE_STOP, S5_MOVE_FWD, 0u, FALSE);
    for (t = 5000u; t <= 8000u; t += 10u) vstep(&ctl, t, 0, 0);
    EXPECT(g_b_halted && !g_b_line_halt && max_pwm_seen == 0u);
    EXPECT(ctl.status.active_command != MOTION_CMD_FOLLOW_LINE);

    /* (f) An obstacle mid barcode turn: S5 takes over and the turn ends. */
    s5_follow(&ctl);
    g_b_turning = true; g_turn_stage = TURN_PIVOT;
    s5_post(S5_REQ_HOLD, S5_MOVE_FWD, 0u, FALSE);
    vstep(&ctl, 6000, 1, 1);
    EXPECT(g_b_in_avoid && !g_b_turning && g_turn_stage == 0u);

    s5_reset();
}
#endif

#if S1_REAL_LINK
/* ---- INT-15 (#51): invalid or corrupted commands, S1 -> VC -> S2 --------
 * Every bad message goes in at the broker end of the real S1 receive path.
 * It must be counted as invalid by S1, never reach the VC's command buffer,
 * and leave the robot exactly as it was: still waiting in Ready, still
 * following, or still paused. A valid command straight after must work. */
#define CMD_TOPIC MQTT_CLIENT_ID "/command"
typedef struct {
    const char *name;
    unsigned char bytes[3];
    unsigned announced, len, chunk;   /* see broker_publish() */
} BAD_COMMAND;
static const BAD_COMMAND bad_commands[] = {
    {"empty payload",                   {0},          0u, 0u, 0u},
    {"2 bytes: START + 0",              {5u, 0u},     2u, 2u, 0u},
    {"2 bytes in 2 chunks: START START",{5u, 5u},     2u, 2u, 1u},
    {"3 bytes: PAUSE x3",               {1u, 1u, 1u}, 3u, 3u, 0u},
    {"announced 1 byte, 2 delivered",   {5u, 0u},     1u, 2u, 0u},
    {"reserved code 6",                 {6u},         1u, 1u, 0u},
    {"reserved code 7",                 {7u},         1u, 1u, 0u},
    {"code 8 (bit 3 only)",             {8u},         1u, 1u, 0u},
    {"START + bit 3 (0x0d)",            {0x0du},      1u, 1u, 0u},
    {"START + bit 7 (0x85)",            {0x85u},      1u, 1u, 0u},
    {"PAUSE + bit 7 (0x81)",            {0x81u},      1u, 1u, 0u},
    {"RESUME + bit 4 (0x12)",           {0x12u},      1u, 1u, 0u},
    {"all ones (0xff)",                 {0xffu},      1u, 1u, 0u},
};
#define BAD_COMMANDS (sizeof(bad_commands) / sizeof(bad_commands[0]))

static S1_COMMAND_STATS stats(void)
{
    S1_COMMAND_STATS s; s1_command_stats_get(&s); return s;
}
static bool counted(unsigned received, unsigned queued, unsigned invalid)
{
    S1_COMMAND_STATS s = stats();
    return s.received == received && s.queued == queued && s.invalid == invalid &&
           s.dropped == 0u && s.received == s.queued + s.invalid + s.dropped;
}
static void send_bad(const BAD_COMMAND *b)
{
    broker_publish(CMD_TOPIC, b->bytes, b->announced, b->len, b->chunk);
}
static bool stopped_motors(const MOTION_CONTROLLER *ctl)
{
    return ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0;
}

static void bad_command_tests(void)
{
    MOTION_CONTROLLER ctl;
    unsigned i;
    uint32_t t;

    /* The link subscribed to the command and map topics at QoS 1. */
    EXPECT(subscribe_count >= 2u && subscribe_qos_ok >= 2u);
    EXPECT(strcmp(subscribed[0], CMD_TOPIC) == 0 &&
           strcmp(subscribed[1], MQTT_CLIENT_ID "/mapping") == 0);

    /* (a) Ready: no bad message starts the robot, not even START with a
     * reserved bit set. Nothing reaches the VC's buffer; the motors stay 0. */
    reset(&ctl);
    g_b_auto_start = false;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!g_b_armed && g_b_ready_reported);
    t = 5010u;
    for (i = 0u; i < BAD_COMMANDS; ++i, t += 10u) {
        send_bad(&bad_commands[i]);
        EXPECT(command_count == 0u);                       /* rejected by S1 */
        EXPECT(counted(i + 1u, 0u, i + 1u));
        fake_imu.sample_count++;
        robot_control_step(&ctl, t, 0, 0);
        motion_control_step(&ctl, t + 5u, 0, 0);
        EXPECT(!g_b_armed && !g_b_start_requested && !enabled && stopped_motors(&ctl));
    }
    send_command(S1_COMMAND_START);                        /* the link still works */
    fake_imu.sample_count++;
    robot_control_step(&ctl, t, 0, 0);
    EXPECT(g_b_armed && enabled && counted(BAD_COMMANDS + 1u, 1u, BAD_COMMANDS));

    /* (b) Following: bad messages change nothing (a corrupted PAUSE is not
     * a PAUSE); a valid PAUSE straight after stops at once. */
    follow(&ctl);
    t = 5050u;
    for (i = 0u; i < BAD_COMMANDS; ++i, t += 10u) {
        send_bad(&bad_commands[i]);
        fake_imu.sample_count++;
        robot_control_step(&ctl, t, 1, 1);
        motion_control_step(&ctl, t + 5u, 1, 1);
        EXPECT(!g_b_paused && !g_b_halted && enabled);
        EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE &&
               ctl.status.left_pwm_percent > 0);
    }
    EXPECT(counted(BAD_COMMANDS, 0u, BAD_COMMANDS));
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, t, 1, 1);
    EXPECT(g_b_paused && !enabled && stopped_motors(&ctl));

    /* (c) Paused: a corrupted RESUME keeps the robot stopped; a valid
     * RESUME then restarts line following. */
    static const unsigned char bad_resume[] = {0x82u, 0x0au, 0x22u, 0x42u};
    for (i = 0u; i < sizeof(bad_resume); ++i) {
        broker_publish(CMD_TOPIC, &bad_resume[i], 1u, 1u, 0u);
        for (uint32_t k = 0u; k < 10u; ++k, t += 10u) {
            fake_imu.sample_count++;
            robot_control_step(&ctl, t, 1, 1);
            motion_control_step(&ctl, t + 5u, 1, 1);
            EXPECT(g_b_paused && !enabled && stopped_motors(&ctl));
        }
    }
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, t, 1, 1);
    EXPECT(!g_b_paused && enabled);
    fake_imu.sample_count++;
    robot_control_step(&ctl, t + 50u, 1, 1);
    motion_control_step(&ctl, t + 60u, 2, 2);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT(counted(BAD_COMMANDS + 2u + sizeof(bad_resume), 2u,
                   BAD_COMMANDS + sizeof(bad_resume)));

    /* (d) A flood of bad messages takes no buffer slot: a valid PAUSE sent
     * after 100 of them is queued (not dropped) and applied. */
    follow(&ctl);
    for (i = 0u; i < 100u; ++i) send_bad(&bad_commands[i % BAD_COMMANDS]);
    EXPECT(command_count == 0u);
    send_command(S1_COMMAND_PAUSE);
    EXPECT(command_count == 1u && counted(101u, 1u, 100u));
    robot_control_step(&ctl, 5050, 1, 1);
    EXPECT(g_b_paused && stopped_motors(&ctl));

    /* (e) Bad and valid interleaved in one control period: only the valid
     * ones reach the VC, in order (STATUS answered, then PAUSE applied). */
    follow(&ctl);
    publish_count = 0u;
    send_bad(&bad_commands[9]);                            /* 0x85 */
    send_command(S1_COMMAND_STATUS);
    send_bad(&bad_commands[1]);                            /* 2 bytes */
    send_command(S1_COMMAND_PAUSE);
    EXPECT(command_count == 2u && counted(4u, 2u, 2u));
    robot_control_step(&ctl, 5050, 1, 1);
    EXPECT(command_count == 0u && g_b_paused && stopped_motors(&ctl) && publish_count >= 1u);

    /* (f) Valid on the wire but not supported by the VC (RESET_STATS,
     * REVERSE): queued, logged, and the robot keeps following. */
    follow(&ctl);
    send_command(S1_COMMAND_RESET_STATS);
    send_command(S1_COMMAND_REVERSE);
    EXPECT(counted(2u, 2u, 0u));
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5050, 1, 1);
    motion_control_step(&ctl, 5055, 1, 1);
    EXPECT(command_count == 0u && !g_b_paused && !g_b_halted &&
           ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    /* (g) Other topics are not commands: a START byte on a wrong topic, or a
     * 1-byte message on the map topic, is not counted, queued or applied. */
    reset(&ctl);
    g_b_auto_start = false;
    robot_control_step(&ctl, 5000, 0, 0);
    static const unsigned char start = S1_COMMAND_START;
    broker_publish(MQTT_CLIENT_ID "/commands", &start, 1u, 1u, 0u);
    broker_publish(MQTT_CLIENT_ID "/comman", &start, 1u, 1u, 0u);
    broker_publish("other-robot/command", &start, 1u, 1u, 0u);
    broker_publish(MQTT_CLIENT_ID "/mapping", &start, 1u, 1u, 0u);
    EXPECT(command_count == 0u && counted(0u, 0u, 0u));
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5010, 0, 0);
    EXPECT(!g_b_armed && !g_b_start_requested && stopped_motors(&ctl));

    printf("INT-15 bad commands: %u bad payloads x Ready/following, %u bad RESUMEs, "
           "flood, interleave, unsupported, wrong topics\n",
           (unsigned)BAD_COMMANDS, (unsigned)sizeof(bad_resume));
}
#endif

int main(void)
{
    MOTION_CONTROLLER ctl;
    reset(&ctl);
    fake_imu.calibrated = FALSE;
    robot_control_step(&ctl, 6000, 0, 0);
    EXPECT(!g_b_armed && ctl.status.left_pwm_percent == 0);

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
            EXPECT(!g_b_turning && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
            continue;
        }
        EXPECT(g_b_turning && !enabled);
        EXPECT(ctl.target_value == (action == LINE_BARCODE_CMD_UTURN ? 180 : 90));
        bool right = action == LINE_BARCODE_CMD_RIGHT;
        EXPECT(ctl.status.active_command == (right ? MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG));
        int ticks = (int)(ctl.target_value * 2) + 1;
        motion_control_step(&ctl, 5050, right ? ticks : 1, right ? 1 : ticks);
        robot_control_step(&ctl, 5051, right ? ticks : 1, right ? 1 : ticks);
        EXPECT(!g_b_turning && enabled);
    }
    /* #53: lifted (state or the one-sample event) stops and stays stopped. */
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_LIFTED; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.event = IMU_EVENT_LIFT; stopped(&ctl, 5041);
    /* Tilt is report-only (#67): the robot keeps following. */
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_TILT;
    fake_imu.event = IMU_EVENT_TILT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!g_b_halted && enabled);
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
        g_b_map_received = with_map != 0u;
        robot_control_step(&ctl, 5041, 1, 1);
        EXPECT(!g_b_halted && enabled && !g_b_turning);
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
        EXPECT(g_b_halted && !enabled);                 /* not yet */
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(!g_b_halted && enabled);                 /* re-armed on its own */

        /* A brief loss of black restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        snapshot.data[0] = 0u; robot_control_step(&ctl, 5500u, 1, 1);
        snapshot.data[0] = 3u | (2u << 3u);
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(g_b_halted);
        robot_control_step(&ctl, 5500u + rearm, 1, 1);
        EXPECT(!g_b_halted && enabled);

        /* Moving the robot by hand (encoders change) restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        robot_control_step(&ctl, 5500u, 5, 5);
        robot_control_step(&ctl, t0 + rearm, 5, 5);
        EXPECT(g_b_halted);
        robot_control_step(&ctl, 5500u + rearm, 5, 5);
        EXPECT(!g_b_halted && enabled);

        /* An IMU (safety) stop never re-arms, even on stable black. */
        follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
        robot_control_step(&ctl, t0, 1, 1);
        EXPECT(g_b_halted && !g_b_line_halt);
        fake_imu.event = IMU_EVENT_NONE;
        for (uint32_t t = t0 + 100u; t <= t0 + 3u * rearm; t += 100u) {
            fake_imu.sample_count++;
            robot_control_step(&ctl, t, 1, 1);
        }
        EXPECT(g_b_halted && ctl.status.left_pwm_percent == 0);
    }
    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    snapshot.data[1] = LINE_BARCODE_FAULT;
    stopped(&ctl, 5041);
    EXPECT(!g_b_turning); /* Newer fault takes precedence over queued turn. */

    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(g_b_turning);
    fake_imu.sample_count++;
    stopped(&ctl, 15042); /* Turn deadline with a fresh IMU sample. */

    follow(&ctl);
    fake_imu.motion_state = IMU_MOTION_HUMP;
    fake_imu.event = IMU_EVENT_HUMP;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!g_b_halted && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

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
    g_b_auto_start = false;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!g_b_armed && !enabled && g_b_ready_reported);
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5050, 0, 0);
    EXPECT(!g_b_armed && !enabled && command_count == 0u); /* drained, not a start */
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    send_command(S1_COMMAND_START);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 0, 0);
    EXPECT(g_b_armed && enabled && g_b_start_requested);
    robot_control_step(&ctl, 5130, 0, 0);
    motion_control_step(&ctl, 5140, 1, 1);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    send_command(S1_COMMAND_START);                       /* already running */
    robot_control_step(&ctl, 5150, 1, 1);
    EXPECT(g_b_armed && command_count == 0u);

    /* A START sent during initialisation is kept until the robot is Ready. */
    reset(&ctl);
    g_b_auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);                  /* before S2_ROBOT_START_MS */
    EXPECT(!g_b_armed && g_b_start_requested && !g_b_ready_reported);
    fake_imu.sample_count++;
    robot_control_step(&ctl, S2_ROBOT_START_MS, 0, 0);
    EXPECT(g_b_armed && enabled);

    /* Polling never blocks and drains at most S1_QUEUE_DEPTH per step. */
    reset(&ctl);
    g_b_auto_start = false;
    for (unsigned i = 0u; i < 10u; ++i) send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 1000, 0, 0);
#if S1_REAL_LINK
    {   /* The real buffer holds S1_QUEUE_DEPTH; S1 drops (and counts) the rest. */
        S1_COMMAND_STATS s; s1_command_stats_get(&s);
        EXPECT(s.received == 10u && s.queued == S1_QUEUE_DEPTH &&
               s.dropped == 10u - S1_QUEUE_DEPTH && s.invalid == 0u);
        EXPECT(command_count == 0u && command_polls == S1_QUEUE_DEPTH);
    }
#else
    EXPECT(command_count == 10u - S1_QUEUE_DEPTH && command_polls == S1_QUEUE_DEPTH);
#endif

    /* ---- #72 MQTT link ------------------------------------------------- */

    /* PAUSE stops at once and holds; RESUME on a healthy line and IMU
     * restarts line following. */
    follow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(g_b_paused && !enabled && !g_b_halted);
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    for (uint32_t t = 5100u; t <= 7000u; t += 100u) {
        fake_imu.sample_count++;
        robot_control_step(&ctl, t, 1, 1);
        motion_control_step(&ctl, t, 1, 1);
        EXPECT(g_b_paused && ctl.status.left_pwm_percent == 0);
    }
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 7050, 1, 1);
    EXPECT(!g_b_paused && enabled);
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
    EXPECT(g_b_paused && !enabled);
    snapshot.data[0] = 3u | (2u << 3u);
    fake_imu.sensor_ok = FALSE;
    send_command(S1_COMMAND_RESUME);
    robot_control_step(&ctl, 5070, 1, 1);
    EXPECT(g_b_paused && !enabled);

    /* PAUSE during a line stop blocks the automatic re-arm until RESUME. */
    {
        const uint32_t t0 = 5041u, rearm = S2_LINE_REARM_MS;
        follow(&ctl); line_stop(&ctl, t0);
        send_command(S1_COMMAND_PAUSE);
        robot_control_step(&ctl, t0 + 10u, 1, 1);
        robot_control_step(&ctl, t0 + 3u * rearm, 1, 1);
        EXPECT(g_b_halted && g_b_paused && !enabled);              /* no re-arm while paused */
        send_command(S1_COMMAND_RESUME);
        fake_imu.sample_count++;
        robot_control_step(&ctl, t0 + 3u * rearm + 1u, 1, 1);
        EXPECT(!g_b_paused && g_b_halted);          /* time paused doesn't count */
        robot_control_step(&ctl, t0 + 4u * rearm, 1, 1);
        EXPECT(g_b_halted && !enabled);
        robot_control_step(&ctl, t0 + 4u * rearm + 1u, 1, 1);
        EXPECT(!g_b_halted && enabled);         /* a full re-arm wait after RESUME */
    }

    /* A safety stop ignores PAUSE and RESUME: it needs a reboot. */
    follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(g_b_halted && !g_b_line_halt);
    fake_imu.event = IMU_EVENT_NONE;
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 1, 1);
    EXPECT(g_b_halted && !g_b_line_halt && !g_b_paused && !enabled);

    /* PAUSE in Ready cancels a pending START. */
    reset(&ctl);
    g_b_auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 1100, 0, 0);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!g_b_armed && !g_b_start_requested);

    /* The MQTT letter map is applied to S3; a later barcode and its mapped
     * direction appear in telemetry. A bad map changes nothing. */
    follow(&ctl);
    EXPECT(applied_map[0] == '\0' && !g_b_map_received);        /* no built-in map */
    send_map("FAZW");
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(g_b_map_received && strcmp(applied_map, "FAZW") == 0);
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
    EXPECT(g_last_letter == 26u);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5061, 1, 1);
    EXPECT(field(S1_FIELD_BARCODE_LETTER) == 26 &&
           field(S1_FIELD_BARCODE) == S1_BARCODE_STRAIGHT); /* Z = straight */
    EXPECT(field(S1_FIELD_MAP_RECEIVED) == 1);             /* #38 */

    /* Telemetry: once a second, STATUS forces one at once; state, line bits,
     * distance, hump and barcode counts come from the live sources. */
    reset(&ctl);
    g_b_auto_start = false;
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
    EXPECT(field(S1_FIELD_LEFT_TICKS) == 100 && field(S1_FIELD_RIGHT_TICKS) == 100); /* #41 */
    EXPECT(field(S1_FIELD_HIGHEST_HUMP_MM) == 23);
    EXPECT(field(S1_FIELD_BARCODE_BLACK) == 15 && field(S1_FIELD_BARCODE_WHITE) == 14);
    EXPECT(field(S1_FIELD_FAULTS) == 0);
    /* #40: no scan yet -> obstacle fields 0; then S5's last scan is copied;
       a corrupt S5 word leaves them 0. */
    EXPECT(field(S1_FIELD_OBSTACLE_CM) == 0 && field(S1_FIELD_SCAN_DEG) == 0 &&
           field(S1_FIELD_OBSTACLE_WIDTH_CM) == 0);
    fake_obstacle = (S5_OBSTACLE_INFO){29u, 85u, 22u};
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6001, 100, 100);
    EXPECT(field(S1_FIELD_OBSTACLE_CM) == 29 && field(S1_FIELD_SCAN_DEG) == 85 &&
           field(S1_FIELD_OBSTACLE_WIDTH_CM) == 22);
    fake_obstacle = (S5_OBSTACLE_INFO){400u, 180u, 400u};
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6001, 100, 100);
    EXPECT(field(S1_FIELD_OBSTACLE_CM) == 400 && field(S1_FIELD_SCAN_DEG) == 180 &&
           field(S1_FIELD_OBSTACLE_WIDTH_CM) == 400);
    fake_obstacle_er = E_PAR;
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6001, 100, 100);
    EXPECT(field(S1_FIELD_OBSTACLE_CM) == 0 && field(S1_FIELD_SCAN_DEG) == 0 &&
           field(S1_FIELD_OBSTACLE_WIDTH_CM) == 0);
    fake_obstacle_er = E_OK; fake_obstacle = (S5_OBSTACLE_INFO){0u, 0u, 0u};
    /* #46: S5's CAL_FAIL / NO_PATH / LINE_LOST reach telemetry, each alone and
       together, without touching the S1 fault field; none -> 0. */
    EXPECT(field(S1_FIELD_S5_FAULTS) == 0);
    {
        static const UW cases[] = {S5_FAULT_CAL_FAIL, S5_FAULT_NO_PATH,
                                   S5_FAULT_LINE_LOST, S5_FAULT_MASK};
        unsigned c;
        for (c = 0u; c < sizeof(cases) / sizeof(cases[0]); ++c) {
            fake_s5_faults = cases[c];
            send_command(S1_COMMAND_STATUS);
            robot_control_step(&ctl, 6001, 100, 100);
            EXPECT(field(S1_FIELD_S5_FAULTS) == (int)cases[c]);
            EXPECT(field(S1_FIELD_FAULTS) == 0);
        }
        fake_s5_faults = 0u;
        send_command(S1_COMMAND_STATUS);
        robot_control_step(&ctl, 6001, 100, 100);
        EXPECT(field(S1_FIELD_S5_FAULTS) == 0);
    }
    _Static_assert(S5_FAULT_CAL_FAIL == S1_S5_FAULT_CAL_FAIL &&
                   S5_FAULT_NO_PATH == S1_S5_FAULT_NO_PATH &&
                   S5_FAULT_LINE_LOST == S1_S5_FAULT_LINE_LOST &&
                   S5_FAULT_MASK == S1_S5_FAULTS_MAX, "S5 and S1 fault bits must match");
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6002, 100, 100);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_IDLE);        /* paused */
    /* #41: raw counts go out as their low 16 bits: -5 -> 0xfffb, and
       70000 wraps to 4464; the PC takes int16 differences. */
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6003, -5, 70000);
    EXPECT(field(S1_FIELD_LEFT_TICKS) == 0xfffb && field(S1_FIELD_RIGHT_TICKS) == 4464);
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
    straddle_lost_tests();   /* #23 */
    follow_bias_tests();     /* #23 */
    alltest_motion_tests();  /* #8: the rest of all_test */
    alltest_robot_tests();
#if S5_REAL_EXECUTOR
    s5_handover_tests();     /* INT-13, #20 */
#endif
#if S1_REAL_LINK
    bad_command_tests();     /* INT-15, #51 */
#endif

    printf("robot host tests: %u checks passed\n", checks);
    return 0;
}

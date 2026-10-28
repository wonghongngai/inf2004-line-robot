/* Execute the combined production coordinator with simulated RTOS APIs. */
#include "robot_host_stubs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define APP_ALL_TEST 1
#define TP_PRC2 2
#define TMO_FEVR (-1)
typedef unsigned int UINT;
typedef struct { uint32_t hi, lo; } SYSTIM;
ER tk_get_tim(SYSTIM *now);
ER tk_dly_tsk(unsigned ms);
ER tk_slp_tsk(TMO timeout);
ID app_start_task(void (*task)(INT, void *), int priority, unsigned core, const char *name);
ER subsystem1_init(void);
ER subsystem1_start(void);
ER subsystem4_init(void);
ER subsystem4_start(void);

#include "../../app_program/subsystem2/all_buddies_test.c"

static IMU_DATA fake_imu;
static line_barcode_msg_t fake_line, fake_event;
static S1_TELEMETRY published;
static int command = -1;
static ER imu_error, line_error, receive_error, enable_error;
static float motor_left, motor_right;
static bool enabled, has_event;
static unsigned checks, packets;
static int32_t encoder_left, encoder_right;
static unsigned distance_sent;
static ER distance_error, publish_error;
#define EXPECT(x) do { assert(x); ++checks; } while (0)

int tm_printf(UB *format, ...) { (void)format; return 0; }
ER tk_get_tim(SYSTIM *now) { now->lo = 0; return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms; return E_OK; }
ER tk_slp_tsk(TMO timeout) { (void)timeout; return E_OK; }
ID app_start_task(void (*task)(INT, void *), int p, unsigned c, const char *n)
{ (void)task; (void)p; (void)c; (void)n; return 1; }
ER subsystem1_init(void) { return E_OK; }
ER subsystem1_start(void) { return E_OK; }
ER subsystem4_init(void) { return E_OK; }
ER subsystem4_start(void) { return E_OK; }
ER drv_motor_init(void) { drv_motor_stop(); return E_OK; }
ER drv_motor_diagnostics(MOTOR_DIAGNOSTICS *d) { memset(d, 0, sizeof(*d)); return E_OK; }
void drv_motor_set(float l, float r) { motor_left = l; motor_right = r; }
void drv_motor_stop(void) { motor_left = motor_right = 0; }
ER drv_encoder_init(void) { encoder_left = encoder_right = 0; return E_OK; }
void drv_encoder_poll(void) { }
void drv_encoder_get_counts(int32_t *l, int32_t *r) { *l = encoder_left; *r = encoder_right; }
ER subsystem4_add_distance_mm(S4_DISTANCE_DELTA_MM mm)
{ distance_sent += mm; return distance_error; }
ER subsystem4_get_latest(IMU_DATA *out) { *out = fake_imu; return imu_error; }
ER line_service_init(void) { return E_OK; }
ER line_service_start(void) { return E_OK; }
ER line_service_calibrate(uint8_t b, uint8_t w)
{ EXPECT(b == 7u && w == 0u); return E_OK; }
ER line_service_set_layout(uint8_t layout)
{ EXPECT(layout == LINE_BARCODE_LAYOUT_TRACK); return E_OK; }
ER line_service_enable(bool on)
{ if (enable_error < E_OK) return enable_error; enabled = on; has_event = false; return E_OK; }
ER line_service_snapshot(line_barcode_msg_t *out) { *out = fake_line; return line_error; }
ER line_service_receive(line_barcode_msg_t *out, TMO timeout)
{
    (void)timeout;
    if (receive_error < E_OK) return receive_error;
    if (!has_event) return E_TMOUT;
    *out = fake_event; has_event = false; return E_OK;
}
INT subsystem1_receive_command(S1_COMMAND *out, TMO timeout)
{
    (void)timeout;
    if (command < 0) return E_TMOUT;
    out->bytes[0] = (unsigned char)command; command = -1;
    return S1_COMMAND_BYTES;
}
ER subsystem1_publish_telemetry(const S1_TELEMETRY *packet)
{ EXPECT(s1_telemetry_valid(packet)); published = *packet; ++packets; return publish_error; }

static ALL_TEST setup(void)
{
    ALL_TEST t = {0};
    fake_imu = (IMU_DATA){.sensor_ok = TRUE, .calibrated = TRUE, .sample_count = 1};
    fake_line = (line_barcode_msg_t){{2u, LINE_BARCODE_FOLLOW, 0u}};
    imu_error = line_error = receive_error = enable_error = E_OK;
    command = -1; enabled = has_event = false; packets = 0;
    encoder_left = encoder_right = 0; distance_sent = 0;
    distance_error = publish_error = E_OK;
    wheel_odometry_init(&t.odometry, 10.0f, 10.0f, 0, 0);
    drv_motor_stop();
    return t;
}

static void step(ALL_TEST *t, uint32_t now)
{ ++fake_imu.sample_count; all_test_step(t, now); }

static ALL_TEST running(void)
{
    ALL_TEST t = setup();
    step(&t, 4999); EXPECT(t.phase == WAITING && motor_left == 0);
    step(&t, 5000); EXPECT(t.phase == SETTLING && enabled && motor_left == 0);
    step(&t, 5030); EXPECT(t.phase == RUNNING && motor_left == 30 && motor_right == 30);
    return t;
}

int main(void)
{
    ALL_TEST t = running();
    fake_line.data[0] = 4; step(&t, 5040);
    EXPECT(motor_left == 15 && motor_right == 30);
    fake_line.data[0] = 1; step(&t, 5050);
    EXPECT(motor_left == 30 && motor_right == 15);
    command = S1_COMMAND_STATUS; step(&t, 5060);
    int value;
    EXPECT(s1_telemetry_get(&published, S1_FIELD_LINE_SENSORS, &value) && value == 4);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_STATE, &value) && value == S1_STATE_FOLLOWING);
    command = S1_COMMAND_PAUSE; step(&t, 5070);
    EXPECT(t.phase == PAUSED && !enabled && motor_left == 0 && motor_right == 0);
    command = S1_COMMAND_RESUME; step(&t, 5080);
    EXPECT(t.phase == SETTLING && motor_left == 0 && enabled);
    step(&t, 5110); EXPECT(t.phase == RUNNING && motor_left == 30 && motor_right == 15);
    command = S1_COMMAND_RESET_STATS; step(&t, 5120);
    EXPECT(t.phase == RUNNING); /* Unsupported reset never restarts a run. */
    fake_line.data[0] = 0; step(&t, 5130);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    fake_line.data[0] = 3; command = S1_COMMAND_RESUME; step(&t, 5140);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    unsigned before = packets;
    command = S1_COMMAND_STATUS; step(&t, 5150);
    EXPECT(packets == before + 1); /* MQTT status remains live after stop. */

    t = running(); fake_imu.motion_state = IMU_MOTION_TILT; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_FAULTS, &value) && (value & S1_FAULT_IMU));
    t = running(); fake_imu.event = IMU_EVENT_COLLISION; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); all_test_step(&t, 5131); /* Same sample count, stale IMU. */
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); ++fake_imu.read_errors; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_FAULTS, &value) && (value & S1_FAULT_IMU));
    command = S1_COMMAND_STATUS; step(&t, 5050);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_FAULTS, &value) && (value & S1_FAULT_IMU));
    t = running(); imu_error = E_PAR; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); line_error = E_TMOUT; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); receive_error = E_PAR; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); fake_line.data[1] = LINE_BARCODE_JUNCTION; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); has_event = true;
    fake_event = (line_barcode_msg_t){{3u, LINE_BARCODE_EVT_JUNCTION << 3u, 0u}};
    step(&t, 5040); EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); has_event = true;
    fake_event = (line_barcode_msg_t){{3u, LINE_BARCODE_EVT_BARCODE << 3u, 25u}};
    step(&t, 5040); EXPECT(t.phase == RUNNING && t.barcode == S1_BARCODE_UTURN);
    command = S1_COMMAND_STATUS; fake_line.data[0] = 4u; step(&t, 5050);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_LINE_SENSORS, &value) && value == 1);
    t = running(); t.duration_ms = 30000u; step(&t, t.started + t.duration_ms);
    EXPECT(t.phase == STOPPED && t.terminal_state == S1_STATE_FINISHED && motor_left == 0);
    t = running(); command = S1_COMMAND_PAUSE; step(&t, 5040);
    t.duration_ms = 30000u; step(&t, t.started + t.duration_ms);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = setup(); fake_imu.calibrated = false; step(&t, 15000);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = setup(); command = S1_COMMAND_PAUSE; step(&t, 1000);
    EXPECT(t.phase == PAUSED && motor_left == 0);
    command = S1_COMMAND_RESUME; step(&t, 1100);
    EXPECT(t.phase == PAUSED && motor_left == 0); /* Startup delay applies to resume. */
    command = S1_COMMAND_RESUME; step(&t, 5000); step(&t, 5030);
    EXPECT(t.phase == RUNNING && motor_left == 30);
    /* Default continuous mode survives the former 30-second deadline. */
    t = running(); step(&t, 40000); step(&t, 65000);
    EXPECT(t.phase == RUNNING && motor_left == 30 && t.duration_ms == 0);
    t = running(); encoder_left = 10; encoder_right = -10; step(&t, 5040);
    EXPECT(distance_sent == 10u && t.distance_mm == 10u);
    command = S1_COMMAND_STATUS; step(&t, 5050);
    EXPECT(s1_telemetry_get(&published, S1_FIELD_DISTANCE_CM, &value) && value == 1);
    command = S1_COMMAND_PAUSE; step(&t, 5060);
    encoder_left += 10; encoder_right -= 10; step(&t, 5070);
    EXPECT(distance_sent == 10u); /* Hand movement while paused is not travel. */
    t = running(); distance_error = E_PAR; encoder_left = encoder_right = 10;
    step(&t, 5040); EXPECT(t.phase == STOPPED && motor_left == 0);
    t = running(); encoder_left = encoder_right = 10000;
    step(&t, 5040); EXPECT(t.phase == STOPPED && distance_sent == 0 && motor_left == 0);
    t = running(); wheel_odometry_init(&t.odometry, 0, 0, 0, 0);
    encoder_left = encoder_right = 10; step(&t, 5040);
    EXPECT(t.phase == RUNNING && !t.odometry.available && distance_sent == 0);
    t = running(); publish_error = E_TMOUT; command = S1_COMMAND_STATUS; step(&t, 5040);
    EXPECT(t.phase == RUNNING && t.telemetry_error == E_TMOUT && motor_left == 30);
    publish_error = E_OK; command = S1_COMMAND_STATUS; step(&t, 5050);
    EXPECT(t.telemetry_error == E_OK);
    /* Enable must wait for a fresh sensor sample, rather than latching a
     * stop because Wi-Fi delayed the first sample beyond a fixed 30 ms. */
    t = setup(); step(&t, 5000); line_error = E_TMOUT; step(&t, 5030);
    EXPECT(t.phase == SETTLING && motor_left == 0 && motor_right == 0);
    line_error = E_OK; step(&t, 5040);
    EXPECT(t.phase == RUNNING && motor_left == 30 && motor_right == 30);
    t = setup(); step(&t, 5000); fake_line.data[1] = LINE_BARCODE_PAUSED;
    step(&t, 5030); EXPECT(t.phase == SETTLING && motor_left == 0);
    fake_line.data[1] = LINE_BARCODE_FOLLOW; step(&t, 5050);
    EXPECT(t.phase == RUNNING && motor_left == 30);
    t = setup(); step(&t, 5000); line_error = E_TMOUT;
    step(&t, 5030); step(&t, 5500);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    t = setup(); step(&t, 5000); line_error = E_PAR; step(&t, 5030);
    EXPECT(t.phase == STOPPED && motor_left == 0);
    /* Every physical line pattern: centre alone goes straight, either
     * edge steers, and missing/ambiguous patterns return zero outputs. */
    const float left[8] = {0,30,30,30,15,0,15,0};
    const float right[8] = {0,15,30,15,30,0,30,0};
    for (unsigned mask = 0; mask < 8u; ++mask) {
        float l, r;
        bool valid = line_follow_output(mask, LINE_BARCODE_LAYOUT_TRACK, 30, 15, &l, &r);
        EXPECT(valid == (mask != 0u && mask != 5u && mask != 7u));
        EXPECT(l == left[mask] && r == right[mask]);
    }
    for (unsigned mask = 0; mask < 8u; ++mask) {
        float l, r;
        bool valid = line_follow_output(mask, LINE_BARCODE_LAYOUT_SIDE, 30, 15, &l, &r);
        EXPECT(valid == ((mask & 3u) != 0u));
        EXPECT(l == ((mask & 3u) == 0 ? 0 : (mask & 3u) == 2 ? 15 : 30));
        EXPECT(r == ((mask & 3u) == 0 ? 0 : (mask & 3u) == 1 ? 15 : 30));
    }
    EXPECT(all_buddies_test_start() == E_OK && motor_left == 0);
    printf("All-buddies coordinator: %u checks passed\n", checks);
    return 0;
}

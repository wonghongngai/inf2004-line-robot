/* Host unit tests for the S3 line service (app_program/subsystem3/line_service.c).
 *
 * The real service code is compiled in unchanged. Only the RTOS and GPIO
 * calls are replaced:
 *   - the sampling task runs on its own thread, in lockstep with the test:
 *     tick() advances the fake clock, sets the GPIO levels and lets the task
 *     run exactly one loop iteration (it then blocks again in tk_slp_tsk);
 *   - the event message buffer is a real bounded FIFO, so overflow can be
 *     tested;
 *   - mutexes are no-ops, which is safe because the task and the test never
 *     run at the same time.
 *
 * Covers S3-U14-U17, U23, U24, U27 (also run on target by subsystem3_test.c)
 * and S3-U43-U46 (service-level fault latching and barcode -> command).
 * Run: sh testing/run_host_tests.sh s3/line_service
 */
#include "robot_host_stubs.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define tm_printf(...) ((void)0)
#include "../../app_program/subsystem3/line_barcode.h"

/* ---- RTOS types and constants used by line_service.c --------------------- */
#define TASK_STACK_SIZE 2048
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define E_SYS (-5)
#define TA_INHERIT 1u
#define TA_TFIFO 2u
#define TA_HLNG 4u
#define TA_STA 8u
#define TA_RNG3 16u
#define TA_ASSPRC 32u
#define TP_PRC1 1u
#define TP_PRC2 2u
#define TK_SUPPORT_SMP 0
#define GPIO_MODE_IN 0u
typedef struct { uint32_t hi, lo; } SYSTIM;
typedef struct { unsigned mtxatr; } T_CMTX;
typedef struct { unsigned mbfatr, bufsz, maxmsz; } T_CMBF;
typedef struct {
    void (*task)(INT, void *);
    int itskpri, stksz;
    unsigned tskatr, assprc;
} T_CTSK;
typedef struct {
    unsigned cycatr, cyctim;
    void (*cychdr)(void *);
} T_CCYC;

/* ---- Fake clock and GPIO ------------------------------------------------- */
static uint32_t g_clock_ms = 1000u;
static uint8_t g_raw;               /* bit2 GP28, bit1 GP27, bit0 GP0 */

static ER tk_get_tim(SYSTIM *now) { now->hi = 0u; now->lo = g_clock_ms; return E_OK; }
static ER gpio_set_pin(unsigned pin, unsigned mode) { (void)pin; (void)mode; return E_OK; }
static unsigned gpio_get_val(unsigned pin)
{
    switch (pin) {
    case 28u: return (g_raw >> 2) & 1u;
    case 27u: return (g_raw >> 1) & 1u;
    default:  return g_raw & 1u;
    }
}

/* ---- Event message buffer: bounded FIFO ---------------------------------- */
/* micro T-Kernel stores each message with a 4-byte size header, rounded to
 * 4 bytes: a 3-byte message takes 8 of the 96 bytes, so 12 fit. */
#define MBF_CAPACITY (96u / 8u)
static line_barcode_msg_t g_mbf[MBF_CAPACITY];
static unsigned g_mbf_head, g_mbf_count;

static ID tk_cre_mbf(const T_CMBF *c) { (void)c; g_mbf_head = g_mbf_count = 0u; return 2; }
static ER tk_snd_mbf(ID id, const void *p, unsigned size, TMO timeout)
{
    (void)id; (void)timeout;
    if (size != sizeof(line_barcode_msg_t) || g_mbf_count == MBF_CAPACITY) return E_TMOUT;
    memcpy(&g_mbf[(g_mbf_head + g_mbf_count) % MBF_CAPACITY], p, size);
    ++g_mbf_count;
    return E_OK;
}
static INT tk_rcv_mbf(ID id, void *p, TMO timeout)
{
    (void)id; (void)timeout;
    if (g_mbf_count == 0u) return E_TMOUT;
    memcpy(p, &g_mbf[g_mbf_head], sizeof(line_barcode_msg_t));
    g_mbf_head = (g_mbf_head + 1u) % MBF_CAPACITY;
    --g_mbf_count;
    return (INT)sizeof(line_barcode_msg_t);
}

/* ---- Mutex, cyclic handler, task control --------------------------------- */
static ID tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
static ER tk_del_mtx(ID id) { (void)id; return E_OK; }
static ER tk_loc_mtx(ID id, TMO t) { (void)id; (void)t; return E_OK; }
static ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
static ID tk_cre_cyc(const T_CCYC *c) { (void)c; return 3; }
static ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
static ER tk_can_wup(ID id) { (void)id; return E_OK; }
static ER tk_ter_tsk(ID id) { (void)id; return E_OK; }
static ER tk_del_tsk(ID id) { (void)id; return E_OK; }

static T_CTSK g_task;
static ID tk_cre_tsk(const T_CTSK *c) { g_task = *c; return 4; }

/* Lockstep handshake: the task runs only between tick() and its next sleep. */
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cond = PTHREAD_COND_INITIALIZER;
static bool g_task_turn;

static ER tk_slp_tsk(TMO timeout)
{
    (void)timeout;
    pthread_mutex_lock(&g_lock);
    g_task_turn = false;
    pthread_cond_broadcast(&g_cond);
    while (!g_task_turn) pthread_cond_wait(&g_cond, &g_lock);
    pthread_mutex_unlock(&g_lock);
    return E_OK;
}

static void *task_thread(void *arg)
{
    (void)arg;
    g_task.task(0, NULL);
    return NULL;
}

static ER tk_sta_tsk(ID id, INT code)
{
    (void)id; (void)code;
    pthread_t thread;
    pthread_mutex_lock(&g_lock);
    g_task_turn = true;              /* until the task reaches its first sleep */
    pthread_mutex_unlock(&g_lock);
    if (pthread_create(&thread, NULL, task_thread, NULL) != 0) return E_SYS;
    pthread_detach(thread);
    pthread_mutex_lock(&g_lock);
    while (g_task_turn) pthread_cond_wait(&g_cond, &g_lock);
    pthread_mutex_unlock(&g_lock);
    return E_OK;
}

#include "../../app_program/subsystem3/line_service.c"

/* S1 stub: records barcode results the sampling task hands over. */
static unsigned g_s1_results;
static S1_BARCODE_RESULT g_s1_last;
ER subsystem1_publish_barcode(const S1_BARCODE_RESULT *result)
{
    if (!s1_barcode_result_valid(result)) return E_PAR;
    g_s1_last = *result;
    ++g_s1_results;
    return E_OK;
}

/* ---- Test helpers -------------------------------------------------------- */
static unsigned g_failures;
#define CHECK(id, cond, what) do { \
    bool ok_ = (cond); \
    printf("[TEST] %s %s: %s\n", (id), ok_ ? "PASS" : "FAIL", (what)); \
    if (!ok_) ++g_failures; \
} while (0)

/* One sampling period: advance the clock by step_ms and run one task loop. */
static void tick_step(uint8_t raw, uint32_t step_ms)
{
    g_raw = raw;
    g_clock_ms += step_ms;
    pthread_mutex_lock(&g_lock);
    g_task_turn = true;
    pthread_cond_broadcast(&g_cond);
    while (g_task_turn) pthread_cond_wait(&g_cond, &g_lock);
    pthread_mutex_unlock(&g_lock);
}

static void hold(uint8_t raw, uint32_t ms)
{
    for (uint32_t t = 0u; t < ms; ++t) tick_step(raw, 1u);
}

static uint8_t state_of(line_barcode_msg_t m) { return m.data[1] & 7u; }
static uint8_t event_of(line_barcode_msg_t m) { return (m.data[1] >> 3) & 7u; }
static uint8_t faults_of(line_barcode_msg_t m) { return (m.data[2] >> LINE_BARCODE_FAULT_SHIFT) & 7u; }

static void drain(void)
{
    line_barcode_msg_t m;
    while (line_service_receive(&m, TMO_POL) == E_OK) { }
}

/* Count queued events of one type; letter/command of the last one. */
static unsigned take_events(uint8_t event, uint8_t *letter, uint8_t *command)
{
    line_barcode_msg_t m;
    unsigned n = 0u;
    while (line_service_receive(&m, TMO_POL) == E_OK) {
        if (event_of(m) == event) {
            ++n;
            if (letter) *letter = m.data[2] & LINE_BARCODE_LETTER_MASK;
            if (command) *command = (m.data[0] >> 6) & 3u;
        }
    }
    return n;
}

/* Sensors (normalised, 1 = black): centred on the line = C and R black. */
#define ON_LINE   3u     /* scanner white, both line-edge sensors black */
#define ON_BAR    7u     /* scanner black too */
#define OFF_LINE  0u     /* everything white: line absent */

/* Restart navigation cleanly: pause (drains, clears faults), resume. */
static void restart(void)
{
    (void)line_service_enable(false);
    (void)line_service_enable(true);
    hold(ON_LINE, 50u);
    drain();
}

/* Code 39 *X* frame, 60 ms narrow / 180 ms wide, for letter 0..25. */
static void scan_letter(uint8_t letter, bool corrupt_start)
{
    static uint16_t const codes[26] = {
        0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu, 0x04cu,
        0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
        0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};
    uint16_t const chars[3] = {0x094u, codes[letter], 0x094u};
    hold(ON_LINE, 700u);                               /* leading quiet zone */
    for (unsigned i = 0u; i < LINE_BARCODE_FRAME_RUNS; ++i) {
        unsigned c = i / 10u, e = i % 10u;
        bool wide = e < 9u && (chars[c] & (1u << (8u - e)));
        uint32_t ms = wide ? 180u : 60u;
        if (corrupt_start && i == 1u) ms = 60u;        /* '*' wide space made narrow */
        hold((i & 1u) ? ON_LINE : ON_BAR, ms);
    }
    hold(ON_LINE, 700u);                               /* trailing quiet zone */
}

/* Cross-junction: all three sensors black, longer than 4.5 narrow widths. */
static void cross_junction(void)
{
    hold(ON_BAR, 400u);
    hold(ON_LINE, 200u);
}

int main(void)
{
    line_barcode_msg_t msg;
    uint8_t cmd = 0xffu, letter = 0xffu;

    if (line_service_init() != E_OK || line_service_start() != E_OK) {
        puts("[TEST] setup FAIL: service did not start");
        return 1;
    }

    /* S3-U16 (part): resume is refused before calibration. */
    bool refused_uncalibrated = line_service_enable(true) == E_OBJ;

    /* S3-U14: calibration needs opposite levels on every sensor. */
    CHECK("S3-U14",
          line_service_calibrate(0u, 0u) == E_PAR
          && line_service_calibrate(7u, 7u) == E_PAR
          && line_service_calibrate(7u, 1u) == E_PAR      /* one sensor same both ways */
          && line_service_calibrate(8u, 7u) == E_PAR      /* out of range */
          && line_service_calibrate(7u, 0u) == E_OK,
          "calibration rejects indistinguishable or out-of-range levels");

    /* S3-U15: null outputs are rejected. */
    CHECK("S3-U15",
          line_service_snapshot(NULL) == E_PAR && line_service_receive(NULL, TMO_POL) == E_PAR,
          "snapshot and receive reject a null output");

    /* S3-U16: the controller can pause and resume navigation. */
    bool paused = line_service_enable(false) == E_OK;
    hold(ON_LINE, 5u);
    paused = paused && line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_PAUSED;
    bool resumed = line_service_enable(true) == E_OK;
    hold(ON_LINE, 50u);
    resumed = resumed && line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FOLLOW;
    CHECK("S3-U16", refused_uncalibrated && paused && resumed,
          "resume refused before calibration; pause and resume succeed");

    /* S3-U17: while paused the heartbeat stays fresh; without samples it goes stale. */
    (void)line_service_enable(false);
    hold(ON_LINE, 40u);
    bool fresh = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_PAUSED;
    g_clock_ms += 31u;                                  /* no sample for 31 ms */
    bool stale = line_service_snapshot(&msg) == E_TMOUT;
    hold(ON_LINE, 1u);
    bool fresh_again = line_service_snapshot(&msg) == E_OK;
    CHECK("S3-U17", fresh && stale && fresh_again,
          "paused service keeps a fresh heartbeat; a 31 ms gap reports E_TMOUT");

    /* S3-U23: set and clear one letter mapping while paused. */
    bool set_one = line_service_map_letter('Z', LINE_BARCODE_CMD_UTURN) == E_OK
                   && line_service_lookup(25u, &cmd) == E_OK && cmd == LINE_BARCODE_CMD_UTURN;
    bool cleared = line_service_clear_map() == E_OK
                   && line_service_lookup(25u, &cmd) == E_NOEXS;
    CHECK("S3-U23", set_one && cleared, "paused controller can set, read back and clear a mapping");

    /* S3-U24: invalid mapping, layout and lookup arguments; no layout change while enabled. */
    bool bad_args = line_service_map_letter('a', 0u) == E_PAR
                    && line_service_map_letter('[', 0u) == E_PAR
                    && line_service_set_layout(2u) == E_PAR
                    && line_service_lookup(26u, &cmd) == E_PAR
                    && line_service_lookup(0u, NULL) == E_PAR;
    (void)line_service_enable(true);
    bool locked_layout = line_service_set_layout(LINE_BARCODE_LAYOUT_SIDE) == E_OBJ;
    (void)line_service_enable(false);
    CHECK("S3-U24", bad_args && locked_layout,
          "invalid mapping/layout/lookup rejected; layout locked while enabled");

    /* S3-U27: four-letter map; a bad message keeps the old map. */
    bool four = line_service_set_mapping('A', 'B', 'C', 'D') == E_OK;
    for (uint8_t i = 0u; i < 4u; ++i)
        four = four && line_service_lookup(i, &cmd) == E_OK && cmd == i;
    bool rejected = line_service_set_mapping('A', 'A', 'C', 'D') == E_PAR
                    && line_service_set_mapping('a', 'B', 'C', 'D') == E_PAR
                    && line_service_set_mapping('E', 'F', 'G', '1') == E_PAR;
    bool kept = line_service_lookup(1u, &cmd) == E_OK && cmd == LINE_BARCODE_CMD_RIGHT
                && line_service_lookup(4u, &cmd) == E_NOEXS;
    CHECK("S3-U27", four && rejected && kept,
          "four-letter map set; duplicate/lowercase/non-letter rejected and old map kept");

    /* S3-U43: a full event queue latches a QUEUE fault until an explicit reset. */
    restart();
    for (unsigned i = 0u; i < MBF_CAPACITY + 4u; ++i) {   /* LOST/FOUND events, never consumed */
        hold(OFF_LINE, 120u);
        hold(ON_LINE, 20u);
    }
    bool q_fault = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FAULT
                   && (faults_of(msg) & LINE_BARCODE_ERR_QUEUE) != 0u;
    hold(ON_LINE, 200u);                                   /* healthy input does not clear it */
    bool q_latched = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FAULT;
    (void)line_service_enable(false);                      /* reset drains the queue */
    bool q_drained = line_service_receive(&msg, TMO_POL) == E_TMOUT;
    (void)line_service_enable(true);
    hold(ON_LINE, 50u);
    bool q_cleared = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FOLLOW
                     && faults_of(msg) == 0u;
    CHECK("S3-U43", q_fault && q_latched && q_drained && q_cleared,
          "event-queue overflow latches QUEUE fault; pause/resume drains and clears it");

    /* S3-U44: a sampling gap over 20 ms latches a TIMING fault until reset. */
    restart();
    tick_step(ON_LINE, 25u);
    bool t_fault = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FAULT
                   && (faults_of(msg) & LINE_BARCODE_ERR_TIMING) != 0u
                   && line_service_fault_interval_ms() == 25u;
    hold(ON_LINE, 100u);
    bool t_latched = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FAULT;
    restart();
    bool t_cleared = line_service_snapshot(&msg) == E_OK && state_of(msg) == LINE_BARCODE_FOLLOW
                     && line_service_fault_interval_ms() == 0u;
    CHECK("S3-U44", t_fault && t_latched && t_cleared,
          "25 ms sampling gap latches TIMING fault with interval recorded; reset clears it");

    /* S3-U45: barcode read through GPIO, then a junction gives the mapped COMMAND. */
    (void)line_service_set_mapping('A', 'F', 'Z', 'W');
    restart();
    scan_letter(0u, false);                                /* A -> LEFT */
    letter = 0xffu;
    unsigned barcodes = take_events(LINE_BARCODE_EVT_BARCODE, &letter, NULL);
    bool read_a = barcodes == 1u && letter == 0u;
    cross_junction();
    letter = cmd = 0xffu;
    unsigned commands = take_events(LINE_BARCODE_EVT_COMMAND, &letter, &cmd);
    CHECK("S3-U45", read_a && commands == 1u && letter == 0u && cmd == LINE_BARCODE_CMD_LEFT,
          "A read through the service, then one LEFT COMMAND at the next junction");

    /* S3-U46: unmapped letter and damaged barcode cause no COMMAND. */
    restart();
    scan_letter(24u, false);                               /* Y: not in the map */
    letter = 0xffu;
    bool read_y = take_events(LINE_BARCODE_EVT_BARCODE, &letter, NULL) == 1u && letter == 24u;
    cross_junction();
    line_barcode_msg_t m;
    unsigned junctions = 0u, cmds = 0u;
    while (line_service_receive(&m, TMO_POL) == E_OK) {
        junctions += event_of(m) == LINE_BARCODE_EVT_JUNCTION;
        cmds += event_of(m) == LINE_BARCODE_EVT_COMMAND;
    }
    bool unmapped_ok = read_y && junctions == 1u && cmds == 0u;
    restart();
    scan_letter(0u, true);                                 /* A with a corrupted start marker */
    unsigned invalid = 0u, letters = 0u;
    while (line_service_receive(&m, TMO_POL) == E_OK) {
        invalid += event_of(m) == LINE_BARCODE_EVT_INVALID;
        letters += event_of(m) == LINE_BARCODE_EVT_BARCODE;
    }
    cross_junction();
    unsigned damaged_cmds = take_events(LINE_BARCODE_EVT_COMMAND, NULL, NULL);
    bool damaged_ok = invalid == 1u && letters == 0u && damaged_cmds == 0u;
    CHECK("S3-U46", unmapped_ok && damaged_ok,
          "unmapped Y gives a plain JUNCTION; damaged barcode gives INVALID and no COMMAND");

    /* S3-U47: every finished capture is handed to S1 for <id>/barcode. */
    restart();
    unsigned before = g_s1_results;
    scan_letter(25u, false);                               /* Z */
    (void)take_events(LINE_BARCODE_EVT_BARCODE, NULL, NULL);
    S1_BARCODE_RESULT valid = g_s1_last;
    bool one_valid = g_s1_results == before + 1u && valid.letter == 'Z' &&
                     valid.runs == S1_BARCODE_RUNS_MAX;
    restart();
    scan_letter(0u, true);                                 /* damaged A */
    while (line_service_receive(&m, TMO_POL) == E_OK) {}
    bool one_invalid = g_s1_results == before + 2u && g_s1_last.letter == 0u &&
                       g_s1_last.id == valid.id + 1u;
    CHECK("S3-U47", one_valid && one_invalid,
          "decoded Z and a damaged capture each give one S1 barcode result; frame id increments");

    printf("%s: %u failure(s)\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}

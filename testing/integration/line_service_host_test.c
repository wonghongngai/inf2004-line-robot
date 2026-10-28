/* Real S3 service configuration/freshness code with a simulated RTOS. */
#include "robot_host_stubs.h"
#include <assert.h>
#include <stdio.h>
#define APP_ALL_TEST 1
#define TK_SUPPORT_SMP 1
#define TASK_STACK_SIZE 2048
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define E_OBJ (-41)
#define E_SYS (-5)
#define TA_INHERIT 1u
#define TA_TFIFO 2u
#define TA_HLNG 4u
#define TA_STA 8u
#define TA_RNG3 16u
#define TA_ASSPRC 32u
#define TP_PRC1 1u
#define TP_PRC2 2u
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

static uint32_t clock_ms;
static T_CTSK created_task;
static unsigned wakeups;
static ID tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
static ID tk_cre_mbf(const T_CMBF *c) { (void)c; return 2; }
static ER tk_del_mtx(ID id) { (void)id; return E_OK; }
static ER tk_loc_mtx(ID id, TMO timeout) { (void)id; (void)timeout; return E_OK; }
static ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
static INT tk_rcv_mbf(ID id, void *p, TMO timeout)
{ (void)id; (void)p; (void)timeout; return E_TMOUT; }
static ER tk_snd_mbf(ID id, const void *p, unsigned size, TMO timeout)
{ (void)id; (void)p; (void)size; (void)timeout; return E_OK; }
static ER tk_get_tim(SYSTIM *now) { now->lo = clock_ms; return E_OK; }
static ER tk_wup_tsk(ID id) { (void)id; ++wakeups; return E_OK; }
static ER tk_slp_tsk(TMO timeout) { (void)timeout; return E_OK; }
static ER tk_can_wup(ID id) { (void)id; return E_OK; }
static ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
static ID tk_cre_cyc(const T_CCYC *c)
{ assert(c->cyctim == 10u); return 3; }
static ER tk_ter_tsk(ID id) { (void)id; return E_OK; }
static ER tk_del_tsk(ID id) { (void)id; return E_OK; }
static ID tk_cre_tsk(const T_CTSK *c) { created_task = *c; return 4; }
static ER gpio_set_pin(unsigned pin, unsigned mode) { (void)pin; (void)mode; return E_OK; }
static unsigned gpio_get_val(unsigned pin) { return pin == 27u; }

#include "../../app_program/subsystem3/line_service.c"

int main(void)
{
    line_barcode_msg_t msg;
    assert(line_service_init() == E_OK);
    assert(line_service_calibrate(7, 0) == E_OK);
    assert(line_service_set_layout(LINE_BARCODE_LAYOUT_TRACK) == E_OK);
    assert(line_service_start() == E_OK);
    assert(created_task.assprc == TP_PRC2 && (created_task.tskatr & TA_ASSPRC));
    assert(created_task.itskpri == LINE_SERVICE_PRIORITY);
    line_service_tick(NULL); assert(wakeups == 1u);
    assert(read_sensors() == 2u); /* GP27 alone: physical centre input. */
    g_latest = line_barcode_pack(2, 2, 0, LINE_BARCODE_FOLLOW, LINE_BARCODE_EVT_STATUS, 0);
    g_sample_ms = 100u;
    clock_ms = 130u;
    assert(line_service_snapshot(&msg) == E_OK && (msg.data[0] & 7u) == 2u);
    clock_ms = 131u;
    assert(line_service_snapshot(&msg) == E_TMOUT); /* Staleness guard retained. */
    assert(line_service_enable(true) == E_OK);
    clock_ms = 110u;
    assert(line_service_snapshot(&msg) == E_OK);
    assert((msg.data[1] & 7u) == LINE_BARCODE_PAUSED); /* Old data cannot arm motors. */
    assert(line_service_set_layout(LINE_BARCODE_LAYOUT_SIDE) == E_OBJ);
    assert(line_service_enable(false) == E_OK);
    puts("Line service: motor-core sampling, physical pins, strict 30ms freshness and enable reset PASS");
    return 0;
}

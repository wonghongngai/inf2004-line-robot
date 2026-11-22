/* Host regression: barcode events use their own nonblocking RTOS queue. */
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#define SUBSYSTEM1_H
#include "../../app_program/subsystem1/communication_protocol.h"
#define LOCAL static
#define EXPORT
#define TM_MQTT 1
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TA_TFIFO 0
typedef int ID, INT, ER, TMO;
typedef unsigned char UB;
typedef struct { int mbfatr, bufsz, maxmsz; } T_CMBF;
static int next_id, sent_id, fail_send;
static unsigned char sent[64];
static ID tk_cre_mbf(const T_CMBF *c) { assert(c->maxmsz > 0); return ++next_id; }
static void tk_del_mbf(ID id) { (void)id; }
static ER tk_snd_mbf(ID id, void *p, int n, TMO timeout)
{
    assert(timeout == TMO_POL);
    if (fail_send) return E_TMOUT;
    sent_id = id; memcpy(sent, p, (size_t)n); return E_OK;
}
static INT tk_rcv_mbf(ID id, void *p, TMO timeout)
{ (void)id; (void)p; (void)timeout; return E_TMOUT; }
#define tm_printf(...) ((void)0)
#define __TM_TMONITOR_H__ /* real LOG_* macros (#98); no kernel console */
#include "../../app_program/app_log.h"
#include "../../app_program/subsystem1/subsystem1.c"
int main(void)
{
    assert(subsystem1_init() == E_OK);
    assert(subsystem1_start() == E_OK);
    assert(barcode_queue != telemetry_queue);
    S1_BARCODE_RESULT result = {1u, 29u, 'Z'};
    assert(subsystem1_publish_barcode(&result) == E_OK);
    assert(sent_id == barcode_queue);
    S1_BARCODE_RESULT copy;
    memcpy(&copy, sent, sizeof(copy));
    assert(copy.id == 1u && copy.letter == 'Z' && copy.runs == 29u);
    result.letter = 0; result.runs = 23;
    assert(subsystem1_publish_barcode(&result) == E_OK);
    fail_send = 1;
    assert(subsystem1_publish_barcode(&result) == E_TMOUT);
    assert(subsystem1_publish_barcode(NULL) == E_PAR);
    result.runs = 30;
    assert(subsystem1_publish_barcode(&result) == E_PAR);
    result.runs = 29; result.letter = '?';
    assert(subsystem1_publish_barcode(&result) == E_PAR);
    puts("PASS: valid/invalid barcode events, dedicated queue, full queue and invalid arguments");

    /* S1-U17 command counters: every message counted once, a full queue
     * drops without blocking, and only a burst per window may log. */
    S1_COMMAND_STATS stats;
    const unsigned char status = S1_COMMAND_STATUS, reserved = S1_COMMAND_MAX + 1u;
    fail_send = 0;
    for (int i = 0; i < 10; ++i) s1_deliver_command(&status, S1_COMMAND_BYTES);
    s1_deliver_command(&reserved, S1_COMMAND_BYTES);
    s1_deliver_command(&status, 0u);
    s1_deliver_command(&status, 2u);
    fail_send = 1;
    for (int i = 0; i < 87; ++i) s1_deliver_command(&status, S1_COMMAND_BYTES);
    s1_deliver_command(NULL, S1_COMMAND_BYTES);   /* not a message: not counted */
    s1_command_stats_get(&stats);
    assert(stats.received == 100u && stats.queued == 10u);
    assert(stats.invalid == 3u && stats.dropped == 87u);
    assert(stats.received == stats.queued + stats.invalid + stats.dropped);
    int logged = 0;
    for (int i = 0; i < 100; ++i) logged += s1_command_log_ok();
    assert(logged == (int)S1_COMMAND_LOG_BURST);
    s1_command_stats_report();
    assert(s1_command_log_ok() == 1);              /* new window */
    puts("PASS: command flood counted (100 = 10 queued + 3 invalid + 87 dropped), log burst limited");
}

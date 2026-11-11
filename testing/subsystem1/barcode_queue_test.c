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
}

/* Private service bridge: no RTOS/SDK typedefs cross this header. */
#ifndef COMMUNICATION_INTERNAL_H
#define COMMUNICATION_INTERNAL_H
#include "communication_protocol.h"
int s1_take_telemetry(unsigned char *payload);
/* Pops one queued barcode result; returns its size, or a negative error
 * (E_TMOUT when the queue is empty). */
int s1_take_barcode(S1_BARCODE_RESULT *result);
void s1_deliver_command(const unsigned char *payload, unsigned int length);
void s1_deliver_mapping(const unsigned char *payload, unsigned int length);

/* S1-U17 command counters. Every message on the command topic is counted
 * once, so received == queued + invalid + dropped. Only the Wi-Fi task
 * updates or reads them, so no lock is needed. */
typedef struct {
    unsigned int received;  /* messages on the command topic */
    unsigned int queued;    /* valid and accepted by the queue */
    unsigned int invalid;   /* wrong length or reserved bits set */
    unsigned int dropped;   /* valid, but the queue was full */
} S1_COMMAND_STATS;
void s1_command_stats_get(S1_COMMAND_STATS *out);
/* Returns 1 if this command message may print its own console lines. Only
 * the first few messages of each report window do; a flood would otherwise
 * spend the Wi-Fi task's time printing. */
int s1_command_log_ok(void);
/* Prints a totals line if the window had more commands than it logged, or
 * any rejected or dropped one, then opens a new window. Called from the
 * MQTT heartbeat (every 2 s). */
void s1_command_stats_report(void);
#endif

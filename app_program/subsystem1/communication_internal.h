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
#endif

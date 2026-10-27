/* Private service bridge: no RTOS/SDK typedefs cross this header. */
#ifndef COMMUNICATION_INTERNAL_H
#define COMMUNICATION_INTERNAL_H
#include "communication_protocol.h"
int s1_take_telemetry(unsigned char *payload);
void s1_deliver_command(const unsigned char *payload, unsigned int length);
#endif

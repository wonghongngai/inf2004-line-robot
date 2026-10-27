/* On-target queue tests: make TEST=1. No broker required. */
#include "subsystem1.h"
#include "communication_internal.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem1_test(void)
{
    S1_TELEMETRY packet;
    S1_COMMAND command;
    unsigned char malformed[2] = {0, 0};
    int value = -1;
    TEST_INIT();
    s1_telemetry_init(&packet);
    TEST_CHECK("S1-U01", sizeof(packet) == 10 && sizeof(command) == 1,
               "binary packet sizes");
    TEST_CHECK("S1-U02", s1_telemetry_get(&packet, S1_FIELD_SPEED_CMS, &value)
               && value == 0, "initial logical speed is zero");
    TEST_CHECK("S1-U03", !s1_telemetry_set(&packet, S1_FIELD_SPEED_CMS, 201),
               "out-of-range speed rejected");
    TEST_CHECK("S1-U04", subsystem1_publish_telemetry(NULL) == E_PAR,
               "null telemetry rejected");
    TEST_CHECK("S1-U05", subsystem1_receive_command(&command, TMO_POL) == E_TMOUT,
               "empty queue does not block");
    malformed[0] = S1_COMMAND_STATUS;
    s1_deliver_command(malformed, 1);
    TEST_CHECK("S1-U06", subsystem1_receive_command(&command, TMO_POL) == 1
               && command.bytes[0] == S1_COMMAND_STATUS,
               "binary zero command survives queue");
    malformed[0] = 4;
    s1_deliver_command(malformed, 1);
    s1_deliver_command(malformed, 2);
    TEST_CHECK("S1-U07", subsystem1_receive_command(&command, TMO_POL) == E_TMOUT,
               "reserved bits and wrong length rejected");
    TEST_SUMMARY("subsystem1");
}
#endif

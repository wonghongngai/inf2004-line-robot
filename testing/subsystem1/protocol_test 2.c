/* Host regression: compile with communication_protocol.c, no SDK needed. */
#include <assert.h>
#include <string.h>
#include "communication_protocol.h"

static const int minima[] = {0,0,0,-200,0,0,0,0,0,0,0};
static const int maxima[] = {7,31,100,200,65535,7,4,255,400,180,400};

int main(void)
{
    S1_TELEMETRY packet, before;
    S1_COMMAND command;
    unsigned int field, other, code;
    int value, decoded, neighbor;
    s1_telemetry_init(&packet);
    assert(s1_telemetry_valid(&packet));
    assert(s1_telemetry_get(&packet, S1_FIELD_SPEED_CMS, &decoded) && decoded == 0);
    for (field = 0; field < S1_FIELD_COUNT; ++field) {
        /* Exhaust every legal value, including all 65536 distance values. */
        for (value = minima[field]; value <= maxima[field]; ++value) {
            before = packet;
            assert(s1_telemetry_set(&packet, (S1_FIELD)field, value));
            assert(s1_telemetry_get(&packet, (S1_FIELD)field, &decoded));
            assert(decoded == value && s1_telemetry_valid(&packet));
            for (other = 0; other < S1_FIELD_COUNT; ++other) {
                if (other != field) {
                    assert(s1_telemetry_get(&before, (S1_FIELD)other, &neighbor));
                    assert(s1_telemetry_get(&packet, (S1_FIELD)other, &decoded));
                    assert(decoded == neighbor);
                }
            }
        }
        before = packet;
        assert(!s1_telemetry_set(&packet, (S1_FIELD)field, minima[field] - 1));
        assert(!s1_telemetry_set(&packet, (S1_FIELD)field, maxima[field] + 1));
        assert(memcmp(&packet, &before, sizeof packet) == 0);
    }
    {
        static const int values[] = {7,31,100,-123,65535,7,4,255,400,180,400};
        static const unsigned char expected[] = {0xff,0xe4,0x26,0xff,0xff,0xe7,0x3f,0x64,0x5a,0xc8};
        s1_telemetry_init(&packet);
        for (field = 0; field < S1_FIELD_COUNT; ++field) {
            assert(s1_telemetry_set(&packet, (S1_FIELD)field, values[field]));
        }
        assert(memcmp(packet.bytes, expected, sizeof expected) == 0);
    }
    for (code = 0; code < 256; ++code) {
        command.bytes[0] = (unsigned char)code;
        assert(s1_command_valid(&command) == (code < 4));
        assert(s1_command_set(&command, code) == (code < 4));
    }
    assert(!s1_telemetry_set(&packet, (S1_FIELD)-1, 0));
    assert(!s1_telemetry_set(0, S1_FIELD_STATE, 0));
    assert(!s1_telemetry_valid(0));
    /* Battery 127 is an unused wire encoding. */
    s1_telemetry_init(&packet);
    packet.bytes[1] |= 127;
    assert(!s1_telemetry_valid(&packet));
    return 0;
}

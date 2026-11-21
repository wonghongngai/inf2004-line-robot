/* Host regression: compile with communication_protocol.c, no SDK needed. */
#include <assert.h>
#include <string.h>
#include "communication_protocol.h"

static const int minima[] = {0,0,0,-200,0,0,0,0,0,0,0,0,0,0,0};
static const int maxima[] = {7,31,100,200,65535,7,4,255,400,180,400,26,1,65535,65535};

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
        static const int values[] = {7,31,100,-123,65535,7,4,255,400,180,400,6,1,15,14};
        static const unsigned char expected[] = {0xff,0xe4,0x26,0xff,0xff,0xe7,0x3f,0x64,0x5a,0xc8,0x26,0x0f,0x00,0x0e,0x00};
        s1_telemetry_init(&packet);
        for (field = 0; field < S1_FIELD_COUNT; ++field) {
            assert(s1_telemetry_set(&packet, (S1_FIELD)field, values[field]));
        }
        assert(memcmp(packet.bytes, expected, sizeof expected) == 0);
    }
    for (code = 0; code < 256; ++code) {
        command.bytes[0] = (unsigned char)code;
        assert(s1_command_valid(&command) == (code <= 5));   /* 5 = START (#48) */
        assert(s1_command_set(&command, code) == (code <= 5));
    }
    assert(!s1_telemetry_set(&packet, (S1_FIELD)-1, 0));
    assert(!s1_telemetry_set(0, S1_FIELD_STATE, 0));
    assert(!s1_telemetry_valid(0));
    /* Battery 127 is an unused wire encoding. */
    s1_telemetry_init(&packet);
    packet.bytes[1] |= 127;
    assert(!s1_telemetry_valid(&packet));
    {
        /* Barcode result packet: every letter x runs, frame wraps to 16 bits. */
        static const unsigned char valid_z[] = {0x01, 0x00, 0xba, 0x03};
        static const unsigned char invalid_ffff[] = {0xff, 0xff, 0xe0, 0x02};
        S1_BARCODE_RESULT in, out;
        S1_BARCODE_PACKET wire, kept;
        unsigned int letter, runs, bit;
        for (letter = 0; letter <= 26; ++letter) {
            for (runs = 0; runs <= S1_BARCODE_RUNS_MAX; ++runs) {
                in.id = 0x12345u + letter * 31u + runs;
                in.runs = (unsigned char)runs;
                in.letter = letter == 0 ? 0 : (unsigned char)('A' + letter - 1);
                assert(s1_barcode_pack(&in, &wire));
                assert(s1_barcode_unpack(&wire, &out));
                assert(out.id == (in.id & S1_BARCODE_FRAME_MASK));
                assert(out.letter == in.letter && out.runs == in.runs);
            }
        }
        in.id = 1u; in.runs = 29u; in.letter = 'Z';
        assert(s1_barcode_pack(&in, &wire));
        assert(memcmp(wire.bytes, valid_z, sizeof valid_z) == 0);
        in.id = 0xffffu; in.runs = 23u; in.letter = 0;
        assert(s1_barcode_pack(&in, &wire));
        assert(memcmp(wire.bytes, invalid_ffff, sizeof invalid_ffff) == 0);
        /* Invalid input is rejected and leaves the packet untouched. */
        kept = wire;
        in.runs = 30u;
        assert(!s1_barcode_pack(&in, &wire));
        in.runs = 29u; in.letter = '?';
        assert(!s1_barcode_pack(&in, &wire));
        assert(!s1_barcode_pack(0, &wire) && !s1_barcode_pack(&in, 0));
        assert(memcmp(&wire, &kept, sizeof wire) == 0);
        /* Each reserved bit, letter 27..31 and runs 30..31 are rejected. */
        for (bit = 26; bit < 32; ++bit) {
            memcpy(wire.bytes, valid_z, sizeof valid_z);
            wire.bytes[bit / 8u] |= (unsigned char)(1u << (bit % 8u));
            assert(!s1_barcode_unpack(&wire, &out));
        }
        memcpy(wire.bytes, valid_z, sizeof valid_z);
        wire.bytes[2] = (unsigned char)((wire.bytes[2] & 0xe0u) | 27u);
        assert(!s1_barcode_unpack(&wire, &out));
        memcpy(wire.bytes, valid_z, sizeof valid_z);
        wire.bytes[2] |= 0xe0u; wire.bytes[3] |= 0x03u;    /* runs = 31 */
        assert(!s1_barcode_unpack(&wire, &out));
        assert(!s1_barcode_unpack(0, &out) && !s1_barcode_unpack(&wire, 0));
    }
    return 0;
}

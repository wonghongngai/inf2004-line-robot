/* Host-only, no SDK: see testing/run_host_tests.sh (s2/motion_protocol). */
#if defined(S2_PROTOCOL_HOST_TEST) && S2_PROTOCOL_HOST_TEST
#include "motion_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static MOTION_STATUS round_trip(const MOTION_STATUS *in)
{
    S2_PACKED_STATUS p;
    MOTION_STATUS out;
    memset(&out, 0xA5, sizeof(out));
    assert(s2_status_pack(&p, in));
    assert(s2_status_unpack(&out, &p));
    return out;
}

static void test_command(void)
{
    S2_PACKED_COMMAND p; MOTION_COMMAND out={MOTION_CMD_NONE,0};
    MOTION_COMMAND c={MOTION_CMD_FORWARD_CM,12.5f};
    assert(sizeof(p)==5);
    assert(s2_command_pack(&p,&c));
    assert(p.data[0]==1 && p.data[1]==0 && p.data[2]==0 && p.data[3]==0x48 && p.data[4]==0x41);
    assert(s2_command_unpack(&out,&p) && out.type==c.type && out.value==c.value);
    for(unsigned i=0;i<256;++i) {
        p.data[0]=(uint8_t)i;
        bool valid=s2_command_unpack(&out,&p);
        assert(valid == (i==1 || i==2 || i==3));
    }
    c=(MOTION_COMMAND){MOTION_CMD_FOLLOW_LINE,-1};
    assert(s2_command_pack(&p,&c) && s2_command_unpack(&out,&p) && out.value==-1);
    c.value=1; assert(s2_command_pack(&p,&c));
    c.value=1.01f; assert(!s2_command_pack(&p,&c));
    c.value=NAN; assert(!s2_command_pack(&p,&c));
    c.value=INFINITY; assert(!s2_command_pack(&p,&c));
    c=(MOTION_COMMAND){MOTION_CMD_FORWARD_CM,0}; assert(!s2_command_pack(&p,&c));
    c.value=100000; assert(s2_command_pack(&p,&c));
    c.value=100001; assert(!s2_command_pack(&p,&c));
    c=(MOTION_COMMAND){MOTION_CMD_STOP,0}; assert(s2_command_pack(&p,&c));
    c.value=1; assert(!s2_command_pack(&p,&c));
}

static void test_status_layout(void)
{
    /* (a) 12 bytes, 95 bits used, exact bit positions. */
    assert(sizeof(S2_PACKED_STATUS) == 12 && S2_STATUS_BITS_USED == 95);
    MOTION_STATUS s = {0};
    S2_PACKED_STATUS p;
    s.state = MOTION_STATE_FAULT; s.active_command = MOTION_CMD_FOLLOW_LINE;
    s.fault = MOTION_FAULT_RIGHT_ENCODER_STALL; s.motion_complete = true;
    s.left_encoder_ticks = 0x1234; s.right_encoder_ticks = -1;
    s.left_speed_cm_s = 18.4f; s.right_speed_cm_s = -3.0f;
    s.distance_cm = 5400.0f; s.left_pwm_percent = 12.5f; s.right_pwm_percent = -18.0f;
    assert(s2_status_pack(&p, &s));
    assert(p.data[0] == (6 | (6 << 3) | (3 << 6)));
    assert(s2_get_bits(p.data, 8, 1) == 1);
    assert(s2_get_bits(p.data, 9, 16) == 0x1234);
    assert(s2_get_bits(p.data, 25, 16) == 0xFFFF);
    assert(s2_get_bits(p.data, 41, 8) == 18);               /* 18.4 rounds to 18 */
    assert(s2_get_bits(p.data, 49, 8) == 0xFD);             /* -3 */
    assert(s2_get_bits(p.data, 57, 16) == 5400);
    assert(s2_get_bits(p.data, 73, 11) == 125);             /* 12.5 % */
    assert(s2_get_bits(p.data, 84, 11) == (2048 - 180));    /* -18.0 % */
    assert(s2_get_bits(p.data, 95, 1) == 0);

    /* (b) round trip of every field at measured values. */
    MOTION_STATUS o = round_trip(&s);
    assert(o.state == s.state && o.active_command == s.active_command &&
           o.fault == s.fault && o.motion_complete);
    assert(o.left_encoder_ticks == 0x1234 && o.right_encoder_ticks == -1);
    assert(o.left_speed_cm_s == 18.0f && o.right_speed_cm_s == -3.0f);
    assert(o.distance_cm == 5400.0f);
    assert(o.left_pwm_percent == 12.5f && o.right_pwm_percent == -18.0f);

    /* (c) every valid state/command/fault/complete combination. */
    for (unsigned st = 0; st <= 6; ++st)
        for (unsigned cm = 0; cm <= 6; ++cm)
            for (unsigned f = 0; f <= 3; ++f) {
                MOTION_STATUS e = {0};
                e.state = (MOTION_STATE)st; e.active_command = (MOTION_COMMAND_TYPE)cm;
                e.fault = (MOTION_FAULT)f; e.motion_complete = (f & 1U) != 0U;
                o = round_trip(&e);
                assert((unsigned)o.state == st && (unsigned)o.active_command == cm &&
                       (unsigned)o.fault == f && o.motion_complete == e.motion_complete);
            }
}

static void test_status_ticks(void)
{
    MOTION_STATUS s = {0};
    MOTION_STATUS o;

    /* (d) the 16-bit field wraps; delta across the wrap is still right. */
    int32_t counts[] = {0, 1, 32767, 32768, 65535, 65536, 1106775, -1, -32768,
                        INT32_MAX, INT32_MIN};
    for (unsigned i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
        s.left_encoder_ticks = counts[i];
        s.right_encoder_ticks = counts[i] ^ 0x5A5A; /* no signed overflow */
        o = round_trip(&s);
        assert(s2_tick_delta(o.left_encoder_ticks, counts[i]) == 0);
        assert(s2_tick_delta(o.right_encoder_ticks, s.right_encoder_ticks) == 0);
        assert(o.left_encoder_ticks >= -32768 && o.left_encoder_ticks <= 32767);
    }
    /* Longest measured run (1,106,775 ticks) seen 1 s apart at the
       highest measured rate (2,080 ticks/s), forwards and backwards. */
    int32_t before = 1106775 - 2080;
    s.left_encoder_ticks = before; MOTION_STATUS a = round_trip(&s);
    s.left_encoder_ticks = 1106775; MOTION_STATUS b = round_trip(&s);
    assert(s2_tick_delta(b.left_encoder_ticks, a.left_encoder_ticks) == 2080);
    assert(s2_tick_delta(a.left_encoder_ticks, b.left_encoder_ticks) == -2080);
    /* Crossing 32767 -> 32768 and INT32_MAX -> INT32_MIN. */
    assert(s2_tick_delta(32768 - 65536, 32767) == 1);
    assert(s2_tick_delta(INT32_MIN, INT32_MAX) == 1);
    /* Largest unambiguous delta is +/-32767; 32768 reads as -32768. */
    assert(s2_tick_delta(32767, 0) == 32767);
    assert(s2_tick_delta(-32767, 0) == -32767);
    assert(s2_tick_delta(32768, 0) == -32768);
    assert(s2_tick_delta(65536, 0) == 0);                   /* full wrap: aliased */
}

static void test_status_saturation(void)
{
    MOTION_STATUS s = {0};
    MOTION_STATUS o;

    /* (e) finite out-of-range values saturate, at and beyond each limit. */
    s.left_speed_cm_s = 127.0f; s.right_speed_cm_s = -128.0f;
    o = round_trip(&s);
    assert(o.left_speed_cm_s == 127.0f && o.right_speed_cm_s == -128.0f);
    s.left_speed_cm_s = 127.6f; s.right_speed_cm_s = -1e30f;
    o = round_trip(&s);
    assert(o.left_speed_cm_s == 127.0f && o.right_speed_cm_s == -128.0f);
    s.left_speed_cm_s = FLT_MAX; s.right_speed_cm_s = -FLT_MAX;
    o = round_trip(&s);
    assert(o.left_speed_cm_s == 127.0f && o.right_speed_cm_s == -128.0f);
    s.left_speed_cm_s = 0.49f; s.right_speed_cm_s = -0.5f;  /* rounding */
    o = round_trip(&s);
    assert(o.left_speed_cm_s == 0.0f && o.right_speed_cm_s == -1.0f);
    s.left_speed_cm_s = 0; s.right_speed_cm_s = 0;

    s.distance_cm = 32767.0f; o = round_trip(&s); assert(o.distance_cm == 32767.0f);
    s.distance_cm = 32768.0f; o = round_trip(&s); assert(o.distance_cm == 32767.0f);
    s.distance_cm = 100000.0f; o = round_trip(&s); assert(o.distance_cm == 32767.0f);
    s.distance_cm = -32768.0f; o = round_trip(&s); assert(o.distance_cm == -32768.0f);
    s.distance_cm = -40000.0f; o = round_trip(&s); assert(o.distance_cm == -32768.0f);
    s.distance_cm = 0;

    s.left_pwm_percent = 100.0f; s.right_pwm_percent = -100.0f;
    o = round_trip(&s);
    assert(o.left_pwm_percent == 100.0f && o.right_pwm_percent == -100.0f);
    s.left_pwm_percent = 150.0f; s.right_pwm_percent = -1e9f;
    o = round_trip(&s);
    assert(o.left_pwm_percent == 100.0f && o.right_pwm_percent == -100.0f);
    s.left_pwm_percent = 0.04f; s.right_pwm_percent = 0.06f; /* 0.1 % steps */
    o = round_trip(&s);
    assert(o.left_pwm_percent == 0.0f && fabsf(o.right_pwm_percent - 0.1f) < 1e-6f);
}

static void test_status_rejects(void)
{
    MOTION_STATUS s = {0};
    MOTION_STATUS o;
    S2_PACKED_STATUS p;

    /* (f) pack rejects reserved enum codes, non-finite numbers and NULL. */
    s.state = (MOTION_STATE)7; assert(!s2_status_pack(&p, &s)); s.state = 0;
    s.active_command = (MOTION_COMMAND_TYPE)7; assert(!s2_status_pack(&p, &s));
    s.active_command = 0;
    s.fault = (MOTION_FAULT)4; assert(!s2_status_pack(&p, &s)); s.fault = 0;
    float bad[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < 3; ++i) {
        MOTION_STATUS t = {0};
        t.left_speed_cm_s = bad[i]; assert(!s2_status_pack(&p, &t)); t.left_speed_cm_s = 0;
        t.right_speed_cm_s = bad[i]; assert(!s2_status_pack(&p, &t)); t.right_speed_cm_s = 0;
        t.distance_cm = bad[i]; assert(!s2_status_pack(&p, &t)); t.distance_cm = 0;
        t.left_pwm_percent = bad[i]; assert(!s2_status_pack(&p, &t)); t.left_pwm_percent = 0;
        t.right_pwm_percent = bad[i]; assert(!s2_status_pack(&p, &t));
    }
    assert(!s2_status_pack(NULL, &s) && !s2_status_pack(&p, NULL));

    /* (g) unpack rejects reserved codes, the reserved bit and PWM beyond
       +/-1000 raw, and leaves the output untouched. */
    assert(s2_status_pack(&p, &s));
    MOTION_STATUS keep = {.state = MOTION_STATE_IDLE, .left_encoder_ticks = 999,
                          .distance_cm = 1.5f, .motion_complete = true};
    o = keep;
    S2_PACKED_STATUS q = p; s2_put_bits(q.data, 0, 3, 7);
    assert(!s2_status_unpack(&o, &q));
    q = p; s2_put_bits(q.data, 3, 3, 7); assert(!s2_status_unpack(&o, &q));
    q = p; s2_put_bits(q.data, 95, 1, 1); assert(!s2_status_unpack(&o, &q));
    q = p; s2_put_bits(q.data, 73, 11, 1001); assert(!s2_status_unpack(&o, &q));
    q = p; s2_put_bits(q.data, 84, 11, 2048 - 1001); assert(!s2_status_unpack(&o, &q));
    q = p; s2_put_bits(q.data, 73, 11, 1000); assert(s2_status_unpack(&o, &q));
    assert(o.left_pwm_percent == 100.0f);
    o = keep; q = p; s2_put_bits(q.data, 0, 3, 7); assert(!s2_status_unpack(&o, &q));
    assert(o.state == keep.state && o.left_encoder_ticks == 999 &&
           o.distance_cm == 1.5f && o.motion_complete);
    assert(!s2_status_unpack(NULL, &p) && !s2_status_unpack(&o, NULL));
}

int main(void)
{
    test_command();
    test_status_layout();
    test_status_ticks();
    test_status_saturation();
    test_status_rejects();
    puts("S2 protocol: command bytes; 12-byte status layout, 16-bit tick wrap, "
         "saturation, reserved values and rejects PASS");
    return 0;
}
#endif

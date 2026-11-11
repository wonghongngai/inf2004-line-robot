/* Host-only: gcc -DS2_PROTOCOL_HOST_TEST=1 motion_protocol_host_test.c */
#if defined(S2_PROTOCOL_HOST_TEST) && S2_PROTOCOL_HOST_TEST
#include "motion_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    S2_PACKED_COMMAND p; MOTION_COMMAND out={MOTION_CMD_NONE,0};
    MOTION_COMMAND c={MOTION_CMD_FORWARD_CM,12.5f};
    assert(sizeof(p)==5 && sizeof(S2_PACKED_STATUS)==30);
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
    MOTION_STATUS s={0}; S2_PACKED_STATUS status;
    s.state=MOTION_STATE_FAULT; s.active_command=MOTION_CMD_FOLLOW_LINE;
    s.fault=MOTION_FAULT_RIGHT_ENCODER_STALL; s.motion_complete=true;
    s.left_encoder_ticks=-123; s.right_encoder_ticks=INT32_MAX;
    s.left_speed_cm_s=12.5f; s.right_pwm_percent=100;
    assert(s2_status_pack(&status,&s));
    assert(status.data[0]==(6 | (6<<3) | (3<<6)) && status.data[1]==1);
    assert(s2_get_u32(status.data+2)==(uint32_t)-123);
    assert(s2_get_u32(status.data+6)==INT32_MAX);
    assert(s2_get_float(status.data+10)==12.5f && s2_get_float(status.data+26)==100);
    s.state=(MOTION_STATE)7; assert(!s2_status_pack(&status,&s));
    puts("S2 protocol: byte layout, reserved values, numeric boundaries and precision PASS");
    return 0;
}
#endif

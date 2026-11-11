/* S2 interface v1: explicit bytes, no compiler enum/padding on the wire.
 * Numeric precision is retained pending measured range/resolution decisions.
 * Requires IEEE binary32 (the RP2040 toolchain); little-endian encoding.
 */
#ifndef S2_MOTION_PROTOCOL_H
#define S2_MOTION_PROTOCOL_H
#include "motion_control.h"
#include <float.h>
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "S2 protocol requires binary32 floats");
typedef struct { uint8_t data[5]; } S2_PACKED_COMMAND;
typedef struct { uint8_t data[30]; } S2_PACKED_STATUS;
static inline void s2_put_u32(uint8_t *p, uint32_t v)
{ for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(v >> (8*i)); }
static inline uint32_t s2_get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static inline void s2_put_float(uint8_t *p, float v)
{ union { float f; uint32_t u; } bits = {.f=v}; s2_put_u32(p,bits.u); }
static inline float s2_get_float(const uint8_t *p)
{ union { float f; uint32_t u; } bits = {.u=s2_get_u32(p)}; return bits.f; }
static inline bool s2_command_valid(const MOTION_COMMAND *c)
{
    if (!c || !(c->value >= -FLT_MAX && c->value <= FLT_MAX)) return false;
    switch(c->type) {
    case MOTION_CMD_STOP: case MOTION_CMD_RESET_FAULT: return c->value == 0;
    case MOTION_CMD_FORWARD_CM: case MOTION_CMD_TURN_LEFT_DEG:
    case MOTION_CMD_TURN_RIGHT_DEG: return c->value > 0 && c->value <= 100000;
    case MOTION_CMD_FOLLOW_LINE: return c->value >= -1 && c->value <= 1;
    default: return false;
    }
}
/* byte 0: command bits 0..2; bits 3..7 reserved zero. */
static inline bool s2_command_pack(S2_PACKED_COMMAND *p, const MOTION_COMMAND *c)
{
    if (!p || !s2_command_valid(c)) return false;
    p->data[0]=(uint8_t)c->type; s2_put_float(p->data+1,c->value); return true;
}
static inline bool s2_command_unpack(MOTION_COMMAND *c, const S2_PACKED_COMMAND *p)
{
    if (!p || !c || (p->data[0]&0xf8)) return false;
    MOTION_COMMAND tmp={(MOTION_COMMAND_TYPE)p->data[0],s2_get_float(p->data+1)};
    if (!s2_command_valid(&tmp)) return false;
    *c=tmp; return true;
}
/* byte 0: state[2:0], active command[5:3], fault[7:6].
 * byte 1: complete[0], remaining bits reserved zero.
 * bytes 2..9: signed encoder bit patterns; 10..29: binary32
 * left/right speed (cm/s), distance (cm), left/right PWM (%).
 */
static inline bool s2_status_pack(S2_PACKED_STATUS *p, const MOTION_STATUS *s)
{
    if (!p || !s || (unsigned)s->state>6 || (unsigned)s->active_command>6 ||
        (unsigned)s->fault>3) return false;
    p->data[0]=(uint8_t)(s->state | (s->active_command<<3) | (s->fault<<6));
    p->data[1]=s->motion_complete ? 1 : 0;
    s2_put_u32(p->data+2,(uint32_t)s->left_encoder_ticks);
    s2_put_u32(p->data+6,(uint32_t)s->right_encoder_ticks);
    s2_put_float(p->data+10,s->left_speed_cm_s);
    s2_put_float(p->data+14,s->right_speed_cm_s);
    s2_put_float(p->data+18,s->distance_cm);
    s2_put_float(p->data+22,s->left_pwm_percent);
    s2_put_float(p->data+26,s->right_pwm_percent); return true;
}
#endif

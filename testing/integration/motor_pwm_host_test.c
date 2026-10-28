/* Register-level regression: run real BSP PWM and motor driver against an
   RP2040 register model initially holding PWM in reset (USE_PTMR=0 boot). */
#include "robot_host_stubs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define __SYS_MACHINE_H__
#define __TK_TKERNEL_H__
#define __TM_TMONITOR_H__
#define __BSP_LIBBSP_H__
#define CPU_RP2040 1
#define SYSCLK 125
#define E_IO (-57)
typedef unsigned UINT;
#include "../../include/sys/sysdepend/cpu/rp2040/sysdef.h"
#include "../../include/bsp/sysdepend/cpu/rp2040/libbsp.h"

static UW in_w(UW addr);
static void out_w(UW addr, UW value);
static void set_w(UW addr, UW mask) { out_w(addr, in_w(addr) | mask); }
static void clr_w(UW addr, UW mask) { out_w(addr, in_w(addr) & ~mask); }
#include "../../lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c"
#include "../../app_program/subsystem2/drv_motor.c"

static struct { UW addr, value; } registers[256];
static unsigned used;
static BOOL stuck_reset, stopped_clock;
static UW *reg(UW addr)
{
    for (unsigned i = 0; i < used; ++i)
        if (registers[i].addr == addr) return &registers[i].value;
    assert(used < 256);
    registers[used].addr = addr;
    return &registers[used++].value;
}
static BOOL pwm_address(UW addr) { return addr >= PWM_BASE && addr < PWM_BASE + 0xa0u; }
static UW in_w(UW addr)
{
    if (addr == RESETS_RESET_DONE)
        return (!stuck_reset && !(*reg(RESETS_RESET) & RESETS_RESET_PWM)) ? RESETS_RESET_PWM : 0;
    if (pwm_address(addr)) {
        if (*reg(RESETS_RESET) & RESETS_RESET_PWM) return 0;
        UW offset = addr - PWM_BASE;
        if (!stopped_clock && offset % 0x14u == PWM_CHx_CTR) {
            UW base = PWM_BASE + (offset / 0x14u) * 0x14u;
            if (*reg(base + PWM_CHx_CSR) & PWM_CH_CSR_EN)
                *reg(addr) = (*reg(addr) + 1) % (*reg(base + PWM_CHx_TOP) + 1);
        }
    }
    return *reg(addr);
}
static void out_w(UW addr, UW value)
{
    if (pwm_address(addr) && (*reg(RESETS_RESET) & RESETS_RESET_PWM)) return;
    *reg(addr) = value;
}
static void boot(void)
{
    memset(registers, 0, sizeof(registers)); used = 0;
    stuck_reset = stopped_clock = FALSE;
    *reg(RESETS_RESET) = RESETS_RESET_PWM | RESETS_RESET_I2C1;
}
int main(void)
{
    MOTOR_DIAGNOSTICS d;
    boot();
    /* Reproduce the old failure: enable/wrap writes do nothing in reset. */
    assert(pwm_set_wrap(8, 1000) == E_OK);
    assert(pwm_set_enabled(8, TRUE) == E_OK);
    assert(drv_motor_diagnostics(&d) == E_IO && !d.left_counting);

    assert(drv_motor_init() == E_OK);
    assert(!(*reg(RESETS_RESET) & RESETS_RESET_PWM));
    assert(*reg(RESETS_RESET) & RESETS_RESET_I2C1); /* Preserve other reset bits. */
    assert(drv_motor_diagnostics(&d) == E_OK);
    assert(in_w(MOTOR_PWM_REG(4, PWM_CHx_DIV)) == 200u);
    assert(in_w(MOTOR_PWM_REG(5, PWM_CHx_DIV)) == 200u);
    assert(d.left_counting && d.right_counting);
    assert(d.left_compare == 0 && d.right_compare == 0);
    drv_motor_set(40, 0);
    assert(drv_motor_diagnostics(&d) == E_OK && d.left_compare == 400 && d.right_compare == 0);
    drv_motor_set(0, 40);
    assert(drv_motor_diagnostics(&d) == E_OK && d.left_compare == 0 && d.right_compare == 400);
    drv_motor_set(40, 40);
    assert(drv_motor_diagnostics(&d) == E_OK && d.left_compare == 400 && d.right_compare == 400);
    drv_motor_stop();
    assert(drv_motor_diagnostics(&d) == E_OK && d.left_compare == 0 && d.right_compare == 0);
    stopped_clock = TRUE;
    assert(drv_motor_diagnostics(&d) == E_IO);
    stopped_clock = FALSE;
    *reg(GPIO_CTRL(8)) = GPIO_CTRL_FUNCSEL_SIO;
    assert(drv_motor_diagnostics(&d) == E_IO);
    boot(); stuck_reset = TRUE;
    assert(drv_motor_init() == E_TMOUT); /* Bounded reset failure. */
    printf("motor PWM register regression: PASS (reset, counters, outputs, faults)\n");
    return 0;
}

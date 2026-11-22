/*
 * s5_host_stubs.h - lets subsystem 5's pure logic build on the PC (S5-U45).
 *
 * Force-included (-include) ahead of every S5 source in the host test.
 * Only the kernel/hardware boundary is replaced: obstacle_logic.c,
 * drv_servo.c, drv_ultrasonic.c and subsystem5_test.c are compiled
 * unchanged.  The hardware
 * calls declared here do nothing (see obstacle_logic_host_test.c), so the
 * host test proves the maths only, never the servo or the sensor.
 */
#ifndef S5_HOST_STUBS_H
#define S5_HOST_STUBS_H

/* T-Kernel types, E_* codes, tm_printf and the real LOG_* macros */
#include "../integration/robot_host_stubs.h"

/* Block the real kernel headers: robot_host_stubs.h supplies the types */
#define __TK_TKERNEL_H__

/* Tells subsystem5_test.c it is on the PC (skips the one target-only check) */
#define S5_HOST_TEST 1

typedef unsigned int RELTIM;
typedef unsigned int UINT;

/* RP2040 registers used by drv_servo.c / drv_ultrasonic.c (same values as
 * include/sys/sysdepend/cpu/rp2040/sysdef.h; on the PC they never matter) */
#define PWM_BASE        (0x40050000UL)
#define PWM_CHx_CSR     (0x00UL)
#define PWM_CHx_DIV     (0x04UL)
#define PWM_CHx_CTR     (0x08UL)
#define PWM_CHx_TOP     (0x10UL)
#define PWM_CH_CSR_EN   (1UL << 0)
#define PADS_BANK0_BASE (0x4001c000UL)
#define GPIO(n)         (PADS_BANK0_BASE + 0x4UL + ((n) * 4UL))
#define GPIO_OD         (1UL << 7)
#define GPIO_MODE_OUT   (1U)

ER   tk_dly_tsk(RELTIM dlytim);
void out_w(UW addr, UW data);
UW   in_w(UW addr);
void set_w(UW addr, UW data);
void clr_w(UW addr, UW data);
/* Same prototypes as include/bsp/sysdepend/cpu/rp2040/libbsp.h */
ER   pwm_set_pin(unsigned int no);
ER   pwm_set_wrap(unsigned int no, UW wrap);
ER   pwm_set_cc(unsigned int no, UW cc);
ER   pwm_set_enabled(unsigned int no, BOOL enable);
ER   gpio_set_pin(UINT no, UINT mode);
ER   gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);

#endif /* S5_HOST_STUBS_H */

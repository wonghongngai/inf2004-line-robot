/*
 * drv_motor.h
 * Buddy 2 Robo Pico motor driver.
 */
#ifndef S2_DRV_MOTOR_H
#define S2_DRV_MOTOR_H

#include "../app.h"

/*
 * Configure the four Robo Pico motor-driver GPIO/PWM outputs.
 */
ER drv_motor_init(void);

/*
 * Set left and right forward motor speed as percentages.
 */
void drv_motor_set(
    float left_percent,
    float right_percent
);

/*
 * Immediately set both motors to zero PWM.
 */
void drv_motor_stop(void);

/* Read actual PWM state with outputs unchanged. Counters must advance even
   while stopped (zero compare). This checks the MCU, not motor power. */
typedef struct {
    UW reset_done;
    UW left_csr, right_csr;
    UW left_compare, right_compare;
    BOOL left_counting, right_counting;
} MOTOR_DIAGNOSTICS;
ER drv_motor_diagnostics(MOTOR_DIAGNOSTICS *out);

#endif /* S2_DRV_MOTOR_H */

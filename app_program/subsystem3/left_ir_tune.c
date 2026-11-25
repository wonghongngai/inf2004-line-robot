#include "left_ir_tune.h"
#include "../subsystem2/drv_motor.h"
#include "line_service.h"

#define LEFT_IR_REPORT_MS 250u

ER left_ir_tune_run(void)
{
    ER result = drv_motor_init();
    if (result != E_OK) return result;

    result = gpio_set_pin(LINE_SERVICE_PIN_LEFT, GPIO_MODE_IN);
    if (result != E_OK) return result;

    unsigned low_samples = 0u;
    unsigned high_samples = 0u;
    unsigned edges = 0u;
    unsigned previous = gpio_get_val(LINE_SERVICE_PIN_LEFT);
    SYSTIM now;
    (void)tk_get_otm(&now);
    UW window_start = now.lo;

    for (;;) {
        (void)tk_dly_tsk(LINE_BARCODE_PERIOD_MS);
        unsigned level = gpio_get_val(LINE_SERVICE_PIN_LEFT);
        low_samples += (level == 0u);
        high_samples += (level == 1u);
        edges += (level != previous);
        previous = level;

        (void)tk_get_otm(&now);
        if (now.lo - window_start >= LEFT_IR_REPORT_MS) {
            tm_printf((UB *)"[LEFT IR GP%u] D0=%u low=%u high=%u edges=%u %s\n",
                      LINE_SERVICE_PIN_LEFT, level, low_samples, high_samples,
                      edges, edges == 0u ? "STABLE" : "CHANGED");
            low_samples = 0u;
            high_samples = 0u;
            edges = 0u;
            window_start = now.lo;
        }
    }
}

/*
 * drv_lsm303dlhc.c
 * Buddy 4 private driver for the ST LSM303DLHC accelerometer + magnetometer.
 *
 * I2C0 is expected to be enabled as device "iica".
 * Accelerometer: 7-bit I2C address 0x19
 * Magnetometer:  7-bit I2C address 0x1E
 */

#include "drv_lsm303dlhc.h"
#include "../../device/include/dev_i2c.h"
#include <stdint.h>

#define LSM303_I2C_DEVNAME "iica"

#define LSM303_ACC_ADDR 0x19U
#define LSM303_MAG_ADDR 0x1EU

/* Accelerometer registers. */
#define LSM303_WHO_AM_I_A     0x0FU
#define LSM303_WHO_AM_I_VALUE 0x33U
#define LSM303_CTRL_REG1_A    0x20U
#define LSM303_CTRL_REG4_A    0x23U
#define LSM303_OUT_X_L_A      0x28U
#define LSM303_OUT_X_H_A      0x29U
#define LSM303_OUT_Y_L_A      0x2AU
#define LSM303_OUT_Y_H_A      0x2BU
#define LSM303_OUT_Z_L_A      0x2CU
#define LSM303_OUT_Z_H_A      0x2DU

/* Magnetometer registers. */
#define LSM303_CRA_REG_M 0x00U
#define LSM303_CRB_REG_M 0x01U
#define LSM303_MR_REG_M  0x02U
#define LSM303_OUT_X_H_M 0x03U
#define LSM303_OUT_X_L_M 0x04U
#define LSM303_OUT_Z_H_M 0x05U
#define LSM303_OUT_Z_L_M 0x06U
#define LSM303_OUT_Y_H_M 0x07U
#define LSM303_OUT_Y_L_M 0x08U

/*
 * Accelerometer configuration:
 * CTRL_REG1_A = 0x57 -> 100 Hz, normal mode, X/Y/Z enabled.
 * CTRL_REG4_A = 0x88 -> block-data-update, +/-2 g, high-resolution mode.
 * At +/-2 g in high-resolution mode, output is 12-bit left-justified and
 * sensitivity is approximately 1 mg/LSB after shifting right by four.
 */
#define LSM303_CTRL1_VALUE 0x57U
#define LSM303_CTRL4_VALUE 0x88U

/* Magnetometer: 30 Hz output, +/-1.3 gauss, continuous-conversion mode. */
#define LSM303_CRA_VALUE 0x14U
#define LSM303_CRB_VALUE 0x20U
#define LSM303_MR_VALUE  0x00U

/* I2C0 pins, as in device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h. */
#define LSM303_SDA_PIN            4U
#define LSM303_SCL_PIN            5U
#define LSM303_BUS_HALF_PERIOD_US 5U /* ~100 kHz while bit-banging */
#define LSM303_BUS_CLOCKS         9U

LOCAL ID   lsm303_dd         = 0;
LOCAL BOOL open_err_reported = FALSE;
LOCAL BOOL who_err_reported  = FALSE;

LOCAL ER read_reg(UW sadr, UW reg, UB * value)
{
    if (lsm303_dd <= 0 || value == NULL)
    {
        return E_PAR;
    }

    return i2c_read_reg(lsm303_dd, sadr, reg, value);
}

LOCAL ER write_reg(UW sadr, UW reg, UB value)
{
    if (lsm303_dd <= 0)
    {
        return E_PAR;
    }

    return i2c_write_reg(lsm303_dd, sadr, reg, value);
}

LOCAL ER read_acc_axis(UW low_reg, UW high_reg, W * value_mg)
{
    UB      lo;
    UB      hi;
    int16_t raw16;
    int16_t raw12;
    ER      er;

    if (value_mg == NULL)
    {
        return E_PAR;
    }

    er = read_reg(LSM303_ACC_ADDR, low_reg, &lo);
    if (er < E_OK)
    {
        return er;
    }

    er = read_reg(LSM303_ACC_ADDR, high_reg, &hi);
    if (er < E_OK)
    {
        return er;
    }

    raw16 = (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    raw12 = (int16_t)(raw16 >> 4);

    /* +/-2 g high-resolution mode: approximately 1 mg per LSB. */
    *value_mg = (W)raw12;
    return E_OK;
}

LOCAL ER read_mag_axis(UW high_reg, UW low_reg, W * value_raw)
{
    UB      hi;
    UB      lo;
    int16_t raw;
    ER      er;

    if (value_raw == NULL)
    {
        return E_PAR;
    }

    er = read_reg(LSM303_MAG_ADDR, high_reg, &hi);
    if (er < E_OK)
    {
        return er;
    }

    er = read_reg(LSM303_MAG_ADDR, low_reg, &lo);
    if (er < E_OK)
    {
        return er;
    }

    raw        = (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    *value_raw = (W)raw;
    return E_OK;
}

EXPORT ER lsm303dlhc_init(void)
{
    UB who_am_i;
    ER er;

    if (lsm303_dd <= 0)
    {
        lsm303_dd =
            tk_opn_dev((CONST UB *)LSM303_I2C_DEVNAME, TD_UPDATE | TD_EXCL);
        if (lsm303_dd < E_OK)
        {
            ER open_er = lsm303_dd;

            lsm303_dd = 0; /* never keep an error code as a descriptor */
            if (!open_err_reported)
            { /* later ones: imu_task's report */
                open_err_reported = TRUE;
                LOG_ERR(TAG_S4_IMU, "failed to open I2C device, er=%d",
                        open_er);
            }
            return open_er;
        }
    }

    er = read_reg(LSM303_ACC_ADDR, LSM303_WHO_AM_I_A, &who_am_i);
    if (er < E_OK)
    {
        /* Reported, rate-limited, by imu_task (report_error). */
        return er;
    }

    if (who_am_i != LSM303_WHO_AM_I_VALUE)
    {
        if (!who_err_reported)
        { /* later ones: imu_task's report */
            who_err_reported = TRUE;
            LOG_ERR(TAG_S4_IMU, "unexpected WHO_AM_I=0x%02x", who_am_i);
        }
        return E_IO;
    }

    er = write_reg(LSM303_ACC_ADDR, LSM303_CTRL_REG1_A, LSM303_CTRL1_VALUE);
    if (er < E_OK)
    {
        return er;
    }

    er = write_reg(LSM303_ACC_ADDR, LSM303_CTRL_REG4_A, LSM303_CTRL4_VALUE);
    if (er < E_OK)
    {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_CRA_REG_M, LSM303_CRA_VALUE);
    if (er < E_OK)
    {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_CRB_REG_M, LSM303_CRB_VALUE);
    if (er < E_OK)
    {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_MR_REG_M, LSM303_MR_VALUE);
    if (er < E_OK)
    {
        return er;
    }

    LOG_INFO(TAG_S4_IMU, "LSM303DLHC initialized");
    return E_OK;
}

EXPORT ER lsm303dlhc_read(LSM303_SAMPLE * sample)
{
    ER er;

    if (sample == NULL)
    {
        return E_PAR;
    }

    er = read_acc_axis(LSM303_OUT_X_L_A, LSM303_OUT_X_H_A, &sample->ax_mg);
    if (er < E_OK)
    {
        return er;
    }

    er = read_acc_axis(LSM303_OUT_Y_L_A, LSM303_OUT_Y_H_A, &sample->ay_mg);
    if (er < E_OK)
    {
        return er;
    }

    er = read_acc_axis(LSM303_OUT_Z_L_A, LSM303_OUT_Z_H_A, &sample->az_mg);
    if (er < E_OK)
    {
        return er;
    }

    /* Magnetometer output order is X, Z, Y and is big-endian. */
    er = read_mag_axis(LSM303_OUT_X_H_M, LSM303_OUT_X_L_M, &sample->mx_raw);
    if (er < E_OK)
    {
        return er;
    }

    er = read_mag_axis(LSM303_OUT_Y_H_M, LSM303_OUT_Y_L_M, &sample->my_raw);
    if (er < E_OK)
    {
        return er;
    }

    er = read_mag_axis(LSM303_OUT_Z_H_M, LSM303_OUT_Z_L_M, &sample->mz_raw);
    if (er < E_OK)
    {
        return er;
    }

    return E_OK;
}

/*
 * I2C bus recovery (S4-U17). A slave that lost a clock edge in the middle of
 * a byte (motor noise, a loose wire) can hold SDA low for ever; the controller
 * then fails every transfer with E_IO or E_TMOUT and no amount of retrying
 * helps. The standard cure: take the pins away from the I2C block, clock SCL
 * up to nine times until the slave releases SDA, then send a STOP.
 *
 * Pins are driven open-drain style (output value 0, toggling only the output
 * enable), so a line is either pulled low or left to the pull-ups. Only
 * imu_task uses I2C0 (opened TD_EXCL) and the controller is disabled between
 * transfers (i2c_trans), so nothing else is on the bus while this runs.
 * Takes about 0.1 ms; no kernel calls, so it also works on a core that can no
 * longer block.
 */
LOCAL void bus_wait_us(UW us)
{
    UW start = in_w(TIMER_TIMERAWL);

    while ((UW)(in_w(TIMER_TIMERAWL) - start) < us)
    {
        /* busy-wait: at most a few microseconds */
    }
}

LOCAL void bus_line(UW pin, BOOL low)
{
    out_w(low ? GPIO_OE_SET : GPIO_OE_CLR, 1UL << pin);
    bus_wait_us(LSM303_BUS_HALF_PERIOD_US);
}

EXPORT BOOL lsm303dlhc_recover_bus(void)
{
    UW   mask = (1UL << LSM303_SDA_PIN) | (1UL << LSM303_SCL_PIN);
    UW   i;
    BOOL sda_free;

    out_w(GPIO_OE_CLR, mask);  /* both released (pull-ups) */
    out_w(GPIO_OUT_CLR, mask); /* when enabled, drive low */
    out_w(GPIO_CTRL(LSM303_SDA_PIN), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(LSM303_SCL_PIN), GPIO_CTRL_FUNCSEL_SIO);
    bus_wait_us(LSM303_BUS_HALF_PERIOD_US);

    for (i = 0U; i < LSM303_BUS_CLOCKS &&
                 (in_w(GPIO_IN) & (1UL << LSM303_SDA_PIN)) == 0U;
         i++)
    {
        bus_line(LSM303_SCL_PIN, TRUE);
        bus_line(LSM303_SCL_PIN, FALSE);
    }

    /* STOP: SDA rises while SCL is high. */
    bus_line(LSM303_SCL_PIN, TRUE);
    bus_line(LSM303_SDA_PIN, TRUE);
    bus_line(LSM303_SCL_PIN, FALSE);
    bus_line(LSM303_SDA_PIN, FALSE);

    sda_free = ((in_w(GPIO_IN) & (1UL << LSM303_SDA_PIN)) != 0U);

    out_w(GPIO_CTRL(LSM303_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(LSM303_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
    return sda_free;
}

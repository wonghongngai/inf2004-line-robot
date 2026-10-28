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

#define LSM303_I2C_DEVNAME          "iica"

#define LSM303_ACC_ADDR             0x19U
#define LSM303_MAG_ADDR             0x1EU

/* Accelerometer registers. */
#define LSM303_WHO_AM_I_A           0x0FU
#define LSM303_WHO_AM_I_VALUE       0x33U
#define LSM303_CTRL_REG1_A          0x20U
#define LSM303_CTRL_REG4_A          0x23U
#define LSM303_OUT_X_L_A            0x28U
#define LSM303_OUT_X_H_A            0x29U
#define LSM303_OUT_Y_L_A            0x2AU
#define LSM303_OUT_Y_H_A            0x2BU
#define LSM303_OUT_Z_L_A            0x2CU
#define LSM303_OUT_Z_H_A            0x2DU

/* Magnetometer registers. */
#define LSM303_CRA_REG_M            0x00U
#define LSM303_CRB_REG_M            0x01U
#define LSM303_MR_REG_M             0x02U
#define LSM303_OUT_X_H_M            0x03U
#define LSM303_OUT_X_L_M            0x04U
#define LSM303_OUT_Z_H_M            0x05U
#define LSM303_OUT_Z_L_M            0x06U
#define LSM303_OUT_Y_H_M            0x07U
#define LSM303_OUT_Y_L_M            0x08U

/*
 * Accelerometer configuration:
 * CTRL_REG1_A = 0x57 -> 100 Hz, normal mode, X/Y/Z enabled.
 * CTRL_REG4_A = 0x88 -> block-data-update, +/-2 g, high-resolution mode.
 * At +/-2 g in high-resolution mode, output is 12-bit left-justified and
 * sensitivity is approximately 1 mg/LSB after shifting right by four.
 */
#define LSM303_CTRL1_VALUE          0x57U
#define LSM303_CTRL4_VALUE          0x88U

/* Magnetometer: 30 Hz output, +/-1.3 gauss, continuous-conversion mode. */
#define LSM303_CRA_VALUE            0x14U
#define LSM303_CRB_VALUE            0x20U
#define LSM303_MR_VALUE             0x00U

LOCAL ID lsm303_dd = 0;

LOCAL ER read_reg(UW sadr, UW reg, UB *value)
{
    if (lsm303_dd <= 0 || value == NULL) {
        return E_PAR;
    }

    return i2c_read_reg(lsm303_dd, sadr, reg, value);
}

LOCAL ER write_reg(UW sadr, UW reg, UB value)
{
    if (lsm303_dd <= 0) {
        return E_PAR;
    }

    return i2c_write_reg(lsm303_dd, sadr, reg, value);
}

LOCAL ER read_acc_axis(UW low_reg, UW high_reg, W *value_mg)
{
    UB lo;
    UB hi;
    int16_t raw16;
    int16_t raw12;
    ER er;

    if (value_mg == NULL) {
        return E_PAR;
    }

    er = read_reg(LSM303_ACC_ADDR, low_reg, &lo);
    if (er < E_OK) {
        return er;
    }

    er = read_reg(LSM303_ACC_ADDR, high_reg, &hi);
    if (er < E_OK) {
        return er;
    }

    raw16 = (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    raw12 = (int16_t)(raw16 >> 4);

    /* +/-2 g high-resolution mode: approximately 1 mg per LSB. */
    *value_mg = (W)raw12;
    return E_OK;
}

LOCAL ER read_mag_axis(UW high_reg, UW low_reg, W *value_raw)
{
    UB hi;
    UB lo;
    int16_t raw;
    ER er;

    if (value_raw == NULL) {
        return E_PAR;
    }

    er = read_reg(LSM303_MAG_ADDR, high_reg, &hi);
    if (er < E_OK) {
        return er;
    }

    er = read_reg(LSM303_MAG_ADDR, low_reg, &lo);
    if (er < E_OK) {
        return er;
    }

    raw = (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    *value_raw = (W)raw;
    return E_OK;
}

EXPORT ER lsm303dlhc_init(void)
{
    UB who_am_i;
    ER er;

    if (lsm303_dd <= 0) {
        lsm303_dd = tk_opn_dev((CONST UB *)LSM303_I2C_DEVNAME,
                               TD_UPDATE | TD_EXCL);
        if (lsm303_dd < E_OK) {
            tm_printf((UB *)"Buddy4: failed to open I2C device, er=%d\n",
                      lsm303_dd);
            return lsm303_dd;
        }
    }

    er = read_reg(LSM303_ACC_ADDR, LSM303_WHO_AM_I_A, &who_am_i);
    if (er < E_OK) {
        tm_printf((UB *)"Buddy4: accelerometer not responding, er=%d\n", er);
        return er;
    }

    if (who_am_i != LSM303_WHO_AM_I_VALUE) {
        tm_printf((UB *)"Buddy4: unexpected WHO_AM_I=0x%02x\n", who_am_i);
        return E_IO;
    }

    er = write_reg(LSM303_ACC_ADDR, LSM303_CTRL_REG1_A, LSM303_CTRL1_VALUE);
    if (er < E_OK) {
        return er;
    }

    er = write_reg(LSM303_ACC_ADDR, LSM303_CTRL_REG4_A, LSM303_CTRL4_VALUE);
    if (er < E_OK) {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_CRA_REG_M, LSM303_CRA_VALUE);
    if (er < E_OK) {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_CRB_REG_M, LSM303_CRB_VALUE);
    if (er < E_OK) {
        return er;
    }

    er = write_reg(LSM303_MAG_ADDR, LSM303_MR_REG_M, LSM303_MR_VALUE);
    if (er < E_OK) {
        return er;
    }

    tm_printf((UB *)"Buddy4: LSM303DLHC initialized\n");
    return E_OK;
}

EXPORT ER lsm303dlhc_read(LSM303_SAMPLE *sample)
{
    ER er;

    if (sample == NULL) {
        return E_PAR;
    }

    er = read_acc_axis(LSM303_OUT_X_L_A, LSM303_OUT_X_H_A,
                       &sample->ax_mg);
    if (er < E_OK) return er;

    er = read_acc_axis(LSM303_OUT_Y_L_A, LSM303_OUT_Y_H_A,
                       &sample->ay_mg);
    if (er < E_OK) return er;

    er = read_acc_axis(LSM303_OUT_Z_L_A, LSM303_OUT_Z_H_A,
                       &sample->az_mg);
    if (er < E_OK) return er;

    /* Magnetometer output order is X, Z, Y and is big-endian. */
    er = read_mag_axis(LSM303_OUT_X_H_M, LSM303_OUT_X_L_M,
                       &sample->mx_raw);
    if (er < E_OK) return er;

    er = read_mag_axis(LSM303_OUT_Y_H_M, LSM303_OUT_Y_L_M,
                       &sample->my_raw);
    if (er < E_OK) return er;

    er = read_mag_axis(LSM303_OUT_Z_H_M, LSM303_OUT_Z_L_M,
                       &sample->mz_raw);
    if (er < E_OK) return er;

    return E_OK;
}

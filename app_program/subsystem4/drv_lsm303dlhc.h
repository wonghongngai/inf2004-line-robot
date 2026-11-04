/*
 * drv_lsm303dlhc.h
 * Buddy 4 private LSM303DLHC driver.
 *
 * Only the Buddy 4 IMU task should call this driver. This follows TEAM.md's
 * one-owning-task-per-peripheral rule.
 */

#ifndef DRV_LSM303DLHC_H
#define DRV_LSM303DLHC_H

#include "subsystem4.h"

typedef struct {
    W ax_mg;
    W ay_mg;
    W az_mg;

    W mx_raw;
    W my_raw;
    W mz_raw;
} LSM303_SAMPLE;

EXPORT ER lsm303dlhc_init(void);
EXPORT ER lsm303dlhc_read(LSM303_SAMPLE *sample);

#endif /* DRV_LSM303DLHC_H */

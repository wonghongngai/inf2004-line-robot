/**
 * @file    obstacle_logic.c
 * @brief   Subsystem 5 decision logic.  See obstacle_logic.h.
 */

#include "subsystem5.h"
#include "obstacle_logic.h"

#define HISTORY_MASK      (0x07U) /* keep 3 bits */
#define CONFIRM_THRESHOLD (2U)    /* 2 of the last 3 */

EXPORT BOOL obstacle_filter_update(UB * p_history, ultrasonic_status_t status,
                                   UW distance_mm)
{
    UB b_close;
    UB count;

    if (NULL == p_history)
    {
        return FALSE;
    }

    if ((ULTRASONIC_TOO_NEAR == status) || (ULTRASONIC_BUSY == status))
    {
        /* Unknown readings, do not change history */
        b_close = 0xFFU; /* marker to skip this */
    }
    else if ((ULTRASONIC_OK == status) && (distance_mm < D_DETECT_MM))
    {
        b_close = 1U;
    }
    else
    {
        b_close = 0U;
    }

    /* shift answer into 3-bit memory */
    if (0xFFU != b_close)
    {
        *p_history = (UB)(((*p_history << 1U) | b_close) & HISTORY_MASK);
    }

    /* count the bits that indicate close to obstacle */
    count = (UB)((*p_history & 1U) + ((*p_history >> 1U) & 1U) +
                 ((*p_history >> 2U) & 1U));

    return (count >= CONFIRM_THRESHOLD) ? TRUE : FALSE;
}
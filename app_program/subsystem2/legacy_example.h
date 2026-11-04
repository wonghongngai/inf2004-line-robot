/* Compatibility for the original sample producer used by S3 and S4.
 * These interfaces are unrelated to robot motion. Remove with the example
 * consumer/monitor when the rest of the team replaces those subsystems.
 */
#ifndef S2_LEGACY_EXAMPLE_H
#define S2_LEGACY_EXAMPLE_H

#include "../app.h"

#define S2_PRI_PRODUCER 15
#define S2_PERIOD_MS    300
#define S2_POOL_COUNT   4

typedef struct {
    T_MSG hdr;
    UW seq;
    UW time_ms;
    INT value;
    INT prc;
} SAMPLE_MSG;

IMPORT ID sem_slots;
IMPORT ID mpf_samples;
IMPORT ER s2_legacy_example_init(void);
IMPORT void producer_task(INT stacd, void *exinf);
IMPORT INT s2_make_value(UW seq);

#endif

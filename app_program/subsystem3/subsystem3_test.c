/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	TEST_INIT();
	line_barcode_test();

	/* S3-U01 and INT-01 retired (#74): they tested the course template's
	   example producer/consumer tasks, which are no longer built. */

    TEST_CHECK("S3-U14", E_PAR == line_service_calibrate(0u, 0u),
               "calibration rejects indistinguishable surfaces");
    TEST_CHECK("S3-U15", E_PAR == line_service_snapshot(NULL),
               "snapshot rejects null output");
    TEST_CHECK("S3-U16", E_OK == line_service_enable(false),
               "controller can pause navigation");
    tk_dly_tsk(30);
    line_barcode_msg_t msg = {{0u, 0u}};
    ER result = line_service_snapshot(&msg);
    TEST_CHECK("S3-U17", (E_OK == result)
               && (LINE_BARCODE_PAUSED == (msg.data[1] & 7u)),
               "paused service keeps a fresh sampling heartbeat");
    TEST_CHECK("S3-U23",
               (E_OK == line_service_map_letter('Z', LINE_BARCODE_CMD_UTURN))
               && (E_OK == line_service_clear_map()),
               "paused controller can set and clear a letter mapping");
    TEST_CHECK("S3-U24",
               (E_PAR == line_service_map_letter('a', 0u))
               && (E_PAR == line_service_set_layout(2u)),
               "invalid mapping and layout rejected");
    TEST_CHECK("S3-U27",
               (E_OK == line_service_set_mapping('A', 'B', 'C', 'D'))
               && (E_PAR == line_service_set_mapping('A', 'A', 'C', 'D'))
               && (E_PAR == line_service_set_mapping('a', 'B', 'C', 'D'))
               && (E_OK == line_service_clear_map()),
               "four-letter mapping set; duplicates/lowercase rejected");
	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

EXPORT void subsystem3_test(void)
{
	APP_STATS	before, after;

	TEST_INIT();
	line_barcode_test();

	/* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
	   At one sample per 300 ms, 2 s should bring at least 5. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");

	/* S3-U01: the statistics are self-consistent */
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");

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

/*
 *	app_log.h
 *	One format for every runtime console message.  Included by app.h;
 *	files that cannot include app.h (lwIP code, whose size_t clashes
 *	with T-Kernel's) include this header on its own.
 *
 *	    [S4:IMU] calibration complete base=(3,-12,1004) mg
 *	    [S1:MQTT] WARN connection status -2, retrying in 5s
 *	    [S2:MOTION] ERR stopped: wheel-distance input failed
 *
 *	LOG_INFO  normal progress and readings
 *	LOG_WARN  something unexpected; the robot carries on or stops safely
 *	LOG_ERR   a call or a device failed
 *
 *	tag is one of the TAG_xxx names below.  fmt must be a string literal
 *	(adjacent literals and string macros are fine) WITHOUT a trailing
 *	"\n": the macro adds it.  Test PASS/FAIL lines keep their own [TEST]
 *	format (test_report.h).
 */

#ifndef APP_LOG_H
#define APP_LOG_H

#include <tm/tmonitor.h>	/* tm_printf() */

#define TAG_APP		"APP"
#define TAG_S1_MQTT	"S1:MQTT"	/* broker link, RX/TX		*/
#define TAG_S1_CMD	"S1:CMD"	/* command / map queues		*/
#define TAG_S1_BUZZER	"S1:BUZZER"
#define TAG_S2_ROBOT	"S2:ROBOT"	/* integrated state machine	*/
#define TAG_S2_BARCODE	"S2:BARCODE"	/* barcode frames seen by S2	*/
#define TAG_S2_MOTION	"S2:MOTION"	/* motion task			*/
#define TAG_S2_MOTOR	"S2:MOTOR"	/* PWM driver			*/
#define TAG_S2_DEMO	"S2:DEMO"	/* MOTION_DEMO=1 sequence	*/
#define TAG_S2_HWTEST	"S2:HWTEST"	/* motor bench test		*/
#define TAG_S2_BENCH	"S2:BENCH"	/* S2-U10/U11 floor bench	*/
#define TAG_S3_LINE	"S3:LINE"	/* line / barcode demo		*/
#define TAG_S3_BARCODE	"S3:BARCODE"	/* barcode service		*/
#define TAG_S4_IMU	"S4:IMU"
#define TAG_S5_OBS	"S5:OBS"	/* obstacle scan and bypass	*/

#define LOG_INFO(tag, fmt, ...) \
	tm_printf((UB*)"[" tag "] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(tag, fmt, ...) \
	tm_printf((UB*)"[" tag "] WARN " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(tag, fmt, ...) \
	tm_printf((UB*)"[" tag "] ERR " fmt "\n", ##__VA_ARGS__)

#endif /* APP_LOG_H */

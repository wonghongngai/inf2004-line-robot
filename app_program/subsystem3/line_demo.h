/** @file line_demo.h
 * @brief Stand-alone bench demo: prints line/barcode/junction events.
 *
 * Stands in for the Vehicle Controller until it exists. It is the ONLY
 * consumer of line_nav_q, so set LINE_DEMO_ENABLE to 0 once the real
 * controller calls line_service_receive().
 */
#ifndef LINE_DEMO_H
#define LINE_DEMO_H

#include "../app.h"

#define LINE_DEMO_ENABLE        1
#define LINE_DEMO_PRIORITY      17      /* below the IR task (16) */

/* Raw GPIO level of every sensor on black: true = reads 1 on black (usual
 * for LM393 IR modules). If the printed L/C/R values are inverted on the
 * bench, change this to false.
 */
#define LINE_DEMO_BLACK_HIGH    true

/* TEMPORARY letter-to-movement mapping until Buddy 1's MQTT message calls
 * line_service_set_mapping(). Four distinct letters 'A'..'Z'.
 */
#define LINE_DEMO_MAP_LEFT      'A'
#define LINE_DEMO_MAP_RIGHT     'B'
#define LINE_DEMO_MAP_STRAIGHT  'C'
#define LINE_DEMO_MAP_UTURN     'Z'

ER line_demo_start(void);

#endif /* LINE_DEMO_H */

/*** end of file ***/

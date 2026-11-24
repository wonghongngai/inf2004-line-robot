/*
 *	subsystem3.c
 *	Subsystem 3 -- line following and barcode: creates the line service's
 *	objects and starts its task.
 */

#include "subsystem3.h"
#include "line_demo.h"

EXPORT ER subsystem3_init(void)
{
    return line_service_init();
}

EXPORT ER subsystem3_start(void)
{
    ER er = line_service_start();

    /* Bench printout of barcode/junction events (no motors).  Not in
	   test builds: the tests pause the service and change the map. */
#if LINE_DEMO_ENABLE && !APP_TEST
    if (er == E_OK)
    {
        er = line_demo_start();
    }
#endif
    return er;
}

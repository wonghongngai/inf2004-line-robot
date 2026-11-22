/* Only the RTOS boundary is replaced; controller/adapter are production C. */
#ifndef ROBOT_HOST_STUBS_H
#define ROBOT_HOST_STUBS_H
#define APP_H
#include <stdint.h>
#include <stddef.h>
typedef int ID, ER, INT, BOOL, TMO;
typedef int32_t W;
typedef uint32_t UW;
typedef int16_t H;
typedef uint16_t UH;
typedef unsigned char UB;
typedef struct { void *next; } T_MSG;
#define IMPORT extern
#define EXPORT
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_NOSPT (-9)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_NOEXS (-42)
#define E_TMOUT (-50)
#define TMO_POL 0
int tm_printf(UB *format, ...);
/* APP_H above skips app.h, which is what brings in the LOG_xxx macros and
 * TAG_xxx names (#98); take them from the real header instead. */
#define APP_LOG_HOST_TEST 1
#include "../../app_program/app_log.h"
#endif

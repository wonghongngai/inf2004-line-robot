/* Only the RTOS boundary is replaced; controller/adapter are production C. */
#ifndef ROBOT_HOST_STUBS_H
#define ROBOT_HOST_STUBS_H
#define APP_H
#include <stdint.h>
#include <stddef.h>
typedef int ID, ER, INT, BOOL, TMO;
typedef int32_t W;
typedef uint32_t UW;
typedef unsigned char UB;
typedef struct { void *next; } T_MSG;
#define IMPORT extern
#define EXPORT
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_PAR (-17)
#define E_TMOUT (-50)
#define TMO_POL 0
int tm_printf(UB *format, ...);
#endif

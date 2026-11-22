/* Host stand-in for T-Kernel's <tm/tmonitor.h>, so app_program/app_log.h
 * (LOG_INFO/WARN/ERR, TAG_xxx) compiles in host tests. Each host test
 * declares or #defines tm_printf itself (robot_host_stubs.h, or a
 * `#define tm_printf(...)` before including the production .c). */
#ifndef HOST_TM_TMONITOR_H
#define HOST_TM_TMONITOR_H
#endif

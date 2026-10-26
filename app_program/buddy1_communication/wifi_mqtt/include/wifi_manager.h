
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

bool wifi_init(void);
bool wifi_connect(void);
bool wifi_is_connected(void);
void wifi_disconnect(void);
void wifi_deinit(void);

#endif

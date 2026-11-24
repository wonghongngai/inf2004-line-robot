/* Buddy 1: Robo Pico on-board piezo buzzer (GP22), used for status beeps. */
#ifndef DRV_BUZZER_H
#define DRV_BUZZER_H
#include "../app.h"

/* 1 = beep once each time Wi-Fi comes up with an IP (also after a reconnect).
 * Set to 0 here, or with -DWIFI_CONNECT_BEEP=0, for a silent robot. The board's
 * mute switch silences it too. */
#ifndef WIFI_CONNECT_BEEP
#define WIFI_CONNECT_BEEP 1
#endif

/* Start a tone for ms milliseconds and return at once; a one-shot alarm
 * handler stops it. Task context only (not an ISR). A new beep restarts the
 * timer. Returns E_OK, E_PAR for ms == 0, or the PWM/alarm setup error. */
IMPORT ER buzzer_beep(UINT duration_ms);

/* Wi-Fi link-up hook (declared in cyw43_utk.h); beeps when enabled above. */
IMPORT void wifi_utk_link_up(void);
#endif

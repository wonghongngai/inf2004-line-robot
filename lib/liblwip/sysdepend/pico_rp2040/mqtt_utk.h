/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/* Service-task entry point, implemented in app_program/subsystem1/mqtt_manager.c.
 * Called each Wi-Fi poll while the link has an IP. Never call from student tasks.
 * Connects/reconnects and drains Buddy 1's telemetry queue without blocking. */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */

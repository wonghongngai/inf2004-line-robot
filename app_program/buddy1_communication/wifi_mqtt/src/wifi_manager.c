
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "wifi_manager.h"

// Replace with your Wi-Fi credentials
#define WIFI_SSID "Alicia"
#define WIFI_PASSWORD "alicia0707"

#define WIFI_TIMEOUT_MS 15000

static bool wifi_initialized = false;

bool wifi_init(void) {
    if (wifi_initialized) {
        return true;
    }

    printf("Initializing Wi-Fi...\n");

    if (cyw43_arch_init() != 0) {
        printf("Wi-Fi initialization failed!\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();
    wifi_initialized = true;

    printf("Wi-Fi initialized!\n");
    return true;
}

bool wifi_connect(void) {
    if (!wifi_initialized) {
        return false;
    }

    if (wifi_is_connected()) {
        return true;
    }

    printf("Connecting to Wi-Fi...\n");

    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        WIFI_TIMEOUT_MS
    );

    if (result != 0) {
        printf("Wi-Fi connection failed: %d\n", result);
        return false;
    }

    printf("Wi-Fi connected successfully!\n");
    return true;
}

bool wifi_is_connected(void) {
    if (!wifi_initialized) {
        return false;
    }

    return cyw43_wifi_link_status(
        &cyw43_state,
        CYW43_ITF_STA
    ) == CYW43_LINK_UP;
}

void wifi_disconnect(void) {
    if (!wifi_initialized) {
        return;
    }

    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    printf("Wi-Fi disconnected.\n");
}

void wifi_deinit(void) {
    if (!wifi_initialized) {
        return;
    }

    cyw43_arch_deinit();
    wifi_initialized = false;
}

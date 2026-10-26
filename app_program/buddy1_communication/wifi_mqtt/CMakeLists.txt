
cmake_minimum_required(VERSION 3.13)

# Pass -DPICO_SDK_PATH=/path/to/pico-sdk or export PICO_SDK_PATH.
if (NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if (NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to an installed Raspberry Pi Pico SDK.")
endif()
if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Pico board")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(buddy1_wifi_mqtt C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(buddy1_wifi_mqtt
    src/main.c
    src/wifi_manager.c
    src/mqtt_manager.c
)

target_include_directories(buddy1_wifi_mqtt PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(buddy1_wifi_mqtt
    pico_lwip_mqtt
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)


pico_enable_stdio_usb(buddy1_wifi_mqtt 1)
pico_enable_stdio_uart(buddy1_wifi_mqtt 0)

pico_add_extra_outputs(buddy1_wifi_mqtt)

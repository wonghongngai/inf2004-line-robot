cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(basic C CXX ASM)

pico_sdk_init()

add_executable(basic
    basic.c
)

target_link_libraries(basic pico_stdlib)

pico_enable_stdio_usb(basic 1)
pico_enable_stdio_uart(basic 0)

pico_add_extra_outputs(basic)

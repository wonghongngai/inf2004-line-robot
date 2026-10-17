cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ultrasonic C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Reusable driver, link this into the main robot app later
add_library(ultrasonic_driver STATIC ultrasonic.c)
target_include_directories(ultrasonic_driver PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ultrasonic_driver PUBLIC pico_stdlib hardware_gpio)

# Standalone test program
add_executable(ultrasonic_test ultrasonic_test.c)
target_link_libraries(ultrasonic_test ultrasonic_driver)
pico_enable_stdio_usb(ultrasonic_test 1)
pico_enable_stdio_uart(ultrasonic_test 0)
pico_add_extra_outputs(ultrasonic_test)

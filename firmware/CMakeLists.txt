cmake_minimum_required(VERSION 3.21)

include(pico_sdk_import.cmake)

project(flight_controller)

pico_sdk_init()

add_subdirectory(compass_cal)
add_subdirectory(fc_quad_gt_cc)
add_subdirectory(motor_test)
add_subdirectory(gps_sniffer)

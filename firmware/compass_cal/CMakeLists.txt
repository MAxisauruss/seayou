# Explicit target name, not ${PROJECT_NAME} - that's set once at the top
# level (project(flight_controller)) and shared with the fc_quad_gt_cc
# subdirectory, which now builds alongside this one; reusing it here would
# collide with that target.
add_executable(compass_calibration
    compass_cal.cpp
)

target_link_libraries(compass_calibration
    pico_stdlib
    hardware_i2c
)

# Not calling pico_add_extra_outputs(): on this machine it pulls in picotool,
# which needs a native host C/C++ compiler to build from source (none is
# installed here). Build the .uf2 manually instead - see ../build_and_flash.md.

pico_enable_stdio_usb(compass_calibration 1)
pico_enable_stdio_uart(compass_calibration 0)
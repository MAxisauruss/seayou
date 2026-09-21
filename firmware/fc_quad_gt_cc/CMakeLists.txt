# Explicit target name, not ${PROJECT_NAME} - see the note in
# ../compass_cal/CMakeLists.txt for why.
add_executable(flight_controller
    fc_quad_gt_cc.cpp
)

target_link_libraries(flight_controller
    pico_stdlib
    hardware_uart
    hardware_irq
    hardware_dma
    hardware_i2c
    hardware_pwm
    hardware_adc
    # Level zero-point storage: the captured sensor-to-frame tilt is kept in
    # the last flash sector so it survives a power cycle (Level_Cal_Save() /
    # Level_Cal_Load() in fc_quad_gt_cc.cpp). hardware_sync is what provides
    # save_and_disable_interrupts() around the erase/program.
    hardware_flash
    hardware_sync
)

# Not calling pico_add_extra_outputs(): on this machine it pulls in picotool,
# which needs a native host C/C++ compiler to build from source (none is
# installed here). Build the .uf2 manually instead - objcopy the .elf to a
# .bin (objcopy ships with the ARM toolchain already in use) then run
# build_uf2.py alongside this file. See ../build_and_flash.md.

# USB is now the host link to the Pi5 (see USB_CDC_* in fc_quad_gt_cc.cpp) -
# this pulls in the TinyUSB device library and its background IRQ task.
pico_enable_stdio_usb(flight_controller 1)
pico_enable_stdio_uart(flight_controller 1)
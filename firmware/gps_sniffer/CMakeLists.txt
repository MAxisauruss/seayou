# Bench diagnostic, not flight firmware - see the header of gps_sniffer.cpp.
add_executable(gps_sniffer
    gps_sniffer.cpp
)

target_link_libraries(gps_sniffer
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_adc
)

# Output over USB CDC so it can be read from the Pi with no extra hardware.
pico_enable_stdio_usb(gps_sniffer 1)
pico_enable_stdio_uart(gps_sniffer 0)

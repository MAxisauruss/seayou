# Explicit target name, not ${PROJECT_NAME} - that is set once at the top
# level and shared with the other subdirectories; reusing it here collides.
add_executable(motor_test
    motor_test.cpp
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
)

# Not calling pico_add_extra_outputs(): picotool needs a native host compiler
# that isn't installed here. Build the .uf2 manually with ../build_uf2.py.

pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)

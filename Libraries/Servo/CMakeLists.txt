add_executable(Servo
        Servo.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(Servo 
pico_stdlib 
hardware_pwm
)

# create map/bin/hex file etc.
pico_add_extra_outputs(Servo)

# add url via pico_set_program_url
example_auto_set_url(Servo)
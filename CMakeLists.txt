cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH "~/pico/pico-sdk")

include(pico_sdk_import.cmake)

project(main C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

add_executable(main 
        src/main.cpp
        src/NRF24
        src/MYSERVO
        src/MPU6050
        src/LEGCLASS
        )

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

pico_enable_stdio_uart(main 0)
pico_enable_stdio_usb(main 1)

target_include_directories(main PUBLIC 
inc/
)

target_link_libraries(main 
pico_stdlib
hardware_pwm
hardware_i2c
hardware_spi
)

pico_add_extra_outputs(main)


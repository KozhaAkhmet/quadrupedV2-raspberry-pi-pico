cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH "~/pico/pico-sdk")

include(pico_sdk_import.cmake)

project(QuadrupedV2 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

add_executable(QuadrupedV2
        src/main.cpp
        src/MPU6050.cpp
        src/Servo.cpp
        src/MPU6050.cpp
        src/LegClass.cpp
        src/Balancing.cpp
        )

pico_set_program_name(QuadrupedV2 "QuadrupedV2")
pico_set_program_version(QuadrupedV2 "0.1")

pico_enable_stdio_uart(QuadrupedV2 0)
pico_enable_stdio_usb(QuadrupedV2 1)

target_include_directories(QuadrupedV2 PUBLIC
        inc/
        )

target_link_libraries(QuadrupedV2
        pico_stdlib
        hardware_pwm
        hardware_i2c
        hardware_spi
        pico_multicore
        )

pico_add_extra_outputs(QuadrupedV2)


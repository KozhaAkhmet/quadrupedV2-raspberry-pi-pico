# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/kozha/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(QuadrupedV2_2 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(QuadrupedV2_2 QuadrupedV2_2.cpp )

pico_set_program_name(QuadrupedV2_2 "QuadrupedV2_2")
pico_set_program_version(QuadrupedV2_2 "0.1")

pico_enable_stdio_uart(QuadrupedV2_2 1)
pico_enable_stdio_usb(QuadrupedV2_2 1)

# Add the standard library to the build
target_link_libraries(QuadrupedV2_2 pico_stdlib
hardware_pwm
)

pico_add_extra_outputs(QuadrupedV2_2)


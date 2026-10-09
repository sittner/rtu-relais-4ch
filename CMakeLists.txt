cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Firmware variant: plain (generic relay board) or heater (heating rod controller)
set(FW_PROFILE "plain" CACHE STRING "Firmware variant")
set_property(CACHE FW_PROFILE PROPERTY STRINGS plain heater)
if(NOT FW_PROFILE MATCHES "^(plain|heater)$")
    message(FATAL_ERROR "FW_PROFILE must be 'plain' or 'heater', got '${FW_PROFILE}'")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rtu-relais-4ch)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings, enables C and ASM
project(${CMAKE_PROJECT_NAME} VERSION 1.0 LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})
message("Firmware variant: " ${FW_PROFILE})

# Use our own linker script instead of the CubeMX generated one
# (after project(), which re-reads the toolchain file)
string(REPLACE "${CMAKE_SOURCE_DIR}/STM32F030C8Tx_FLASH.ld" "${CMAKE_SOURCE_DIR}/App/STM32F030C8Tx_app.ld"
    CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS}")
if(NOT CMAKE_C_LINK_FLAGS MATCHES "App/STM32F030C8Tx_app.ld")
    message(FATAL_ERROR "Linker script override failed, check cmake/gcc-arm-none-eabi.cmake")
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME ${CMAKE_PROJECT_NAME}-${FW_PROFILE}
    LINK_DEPENDS ${CMAKE_SOURCE_DIR}/App/STM32F030C8Tx_app.ld
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app.c
    App/Src/led.c
    App/Src/relay.c
    App/Src/profile_${FW_PROFILE}.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    FW_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    FW_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    $<$<STREQUAL:${FW_PROFILE},heater>:FW_PROFILE_HEATER>
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate .hex and .bin next to the .elf
set(FW_IMAGE ${CMAKE_PROJECT_NAME}-${FW_PROFILE})
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FW_IMAGE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FW_IMAGE}.bin
    BYPRODUCTS ${FW_IMAGE}.hex ${FW_IMAGE}.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Flash via ST-Link: cmake --build --preset <preset> --target flash
set(ST_FLASH st-flash CACHE STRING "st-flash executable")
add_custom_target(flash
    COMMAND ${ST_FLASH} --reset write ${FW_IMAGE}.bin 0x8000000
    DEPENDS ${CMAKE_PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    USES_TERMINAL
)

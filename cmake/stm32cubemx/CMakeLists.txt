cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F030x8
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Inc
    ../../Drivers/STM32F0xx_HAL_Driver/Inc
    ../../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32F0xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Src/main.c
    ../../Src/gpio.c
    ../../Src/iwdg.c
    ../../Src/usart.c
    ../../Src/stm32f0xx_it.c
    ../../Src/stm32f0xx_hal_msp.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_iwdg.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_exti.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
    ../../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c
    ../../Src/system_stm32f0xx.c
    ../../Src/sysmem.c
    ../../Src/syscalls.c
    ../../startup_stm32f030x8.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



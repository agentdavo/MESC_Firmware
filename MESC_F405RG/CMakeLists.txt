cmake_minimum_required(VERSION 3.16.4)
project(MESC_F405RG C CXX ASM)

#set(CMAKE_C_STANDARD 11)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/stm32.cmake")

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(compile_definitions "${device_name}")
#if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
#    list(APPEND compile_definitions "DEBUG" "_DEBUG")
#else()
#    list(APPEND compile_definitions "NDEBUG")
#endif()

add_compile_options(
        # CPU specific
        "-mthumb"
        "-mcpu=cortex-m4"
        "-mfloat-abi=hard"
        # Non-CPU specific
        "--specs=nano.specs"
        "-ffunction-sections"
        "-fdata-sections"
        "-fstack-usage"
        # Other options
        "-g3"
        "-c"
        "-Os"
        "-Wall")

add_compile_definitions(
        USE_HAL_DRIVER
        STM32F405xx
        USE_TTERM)

include_directories(
        ../MESC_Common
        ../MESC_Common/Inc
        ../MESC_RTOS
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Core/Inc
        USB_DEVICE/App
        USB_DEVICE/Target)

message(STATUS "CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")

file(GLOB_RECURSE PLATFORM_SOURCES *.c *.s)

add_executable(${PROJECT_NAME}
        ${PLATFORM_SOURCES}
#        Core/Startup/startup_stm32f405rgtx.s
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
#        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
#
#        Middlewares/Third_Party/FreeRTOS/Source/croutine.c
#        Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
#        Middlewares/Third_Party/FreeRTOS/Source/list.c
#        Middlewares/Third_Party/FreeRTOS/Source/queue.c
#        Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
#        Middlewares/Third_Party/FreeRTOS/Source/tasks.c
#        Middlewares/Third_Party/FreeRTOS/Source/timers.c
#        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
#        Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
#        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
#
#        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
#        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
#        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
#        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
#
#        Core/Src/freertos.c
#        Core/Src/main.c
#        Core/Src/MESC_HAL_LLD.c
#        Core/Src/MESChw_setup.c
#        Core/Src/stm32f4xx_hal_msp.c
#        Core/Src/stm32f4xx_hal_timebase_tim.c
#        Core/Src/stm32f4xx_it.c
#        Core/Src/syscalls.c
#        Core/Src/sysmem.c
#        Core/Src/system_stm32f4xx.c
#        USB_DEVICE/App/usb_device.c
#        USB_DEVICE/App/usbd_desc.c
#        USB_DEVICE/App/usbd_cdc_if.c
#        USB_DEVICE/Target/usbd_conf.c

        ../MESC_Common/Src/fonts.c
        ../MESC_Common/Src/MESC_BLDC.c
        ../MESC_Common/Src/MESC_Comms.c
        ../MESC_Common/Src/MESCbat.c
        ../MESC_Common/Src/MESCBLDC.c
        ../MESC_Common/Src/MESCcli.c
        ../MESC_Common/Src/MESCerror.c
        ../MESC_Common/Src/MESCflash.c
        ../MESC_Common/Src/MESCfnv.c
        ../MESC_Common/Src/MESCfoc.c
        ../MESC_Common/Src/MESCmotor.c
        ../MESC_Common/Src/MESCmotor_state.c
        ../MESC_Common/Src/MESCposition.c
        ../MESC_Common/Src/MESCprofile.c
        ../MESC_Common/Src/MESCsin_lut.c
        ../MESC_Common/Src/MESCspeed.c
        ../MESC_Common/Src/MESCtemp.c
        ../MESC_Common/Src/MESCuart.c
        ../MESC_Common/Src/MESCui.c
        ../MESC_Common/Src/MPU6050.c
        ../MESC_Common/Src/Simple_coms.c
        ../MESC_Common/Src/ssd1306.c

        ../MESC_RTOS/Tasks/app_template.c
        ../MESC_RTOS/Tasks/apps.c
        ../MESC_RTOS/Tasks/cana.c
        ../MESC_RTOS/Tasks/init.c
        ../MESC_RTOS/Tasks/task_can.c
        ../MESC_RTOS/Tasks/task_cli.c
        ../MESC_RTOS/Tasks/task_overlay.c
        ../MESC_RTOS/Tasks/top.c

        ../MESC_RTOS/Common/RTOS_flash.c
        ../MESC_RTOS/TTerm/Core/TTerm.c
        ../MESC_RTOS/TTerm/Core/TTerm_AC.c
        ../MESC_RTOS/TTerm/Core/TTerm_cmd.c
        ../MESC_RTOS/TTerm/Core/TTerm_cwd.c
        ../MESC_RTOS/TTerm/Core/TTerm_fnv.c
        ../MESC_RTOS/TTerm/Core/TTerm_var.c

        ../MESC_RTOS/MESC/calibrate.c
        ../MESC_RTOS/MESC/hfi.c
        ../MESC_RTOS/MESC/MESCinterface.c
        )

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
    "-mthumb \
    -mcpu=cortex-m4 -mfloat-abi=hard \
	-T\"${CMAKE_CURRENT_LIST_DIR}/STM32F405RGTX_FLASH.ld\" \
	-Wl,-Map=\"${PROJECT_NAME}.map\" \
	-Wl,--gc-sections \
	-static \
	--specs=nano.specs \
	-Wl,--start-group -lc -lm -Wl,--end-group")

# The OBJCOPY utility will separate the EEPROM section from the main program
add_custom_command(TARGET "${PROJECT_NAME}" POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" ARGS -j .eeprom "$<TARGET_FILE:${PROJECT_NAME}>" -O ihex "eeprom.hex"
        COMMAND "${CMAKE_OBJCOPY}" ARGS -R .eeprom "$<TARGET_FILE:${PROJECT_NAME}>" "program.elf"
        BYPRODUCTS "eeprom.hex" "program.elf")
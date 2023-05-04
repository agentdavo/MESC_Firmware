cmake_minimum_required(VERSION 3.16.4)
project(MESC_RZT2M C CXX ASM)

#set(CMAKE_C_STANDARD 11)

#set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/gcc.cmake")
message(STATUS "RASC_EXE_PATH=${RASC_EXE_PATH}")

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(compile_definitions "${device_name}")
#if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
#    list(APPEND compile_definitions "DEBUG" "_DEBUG")
#else()
#    list(APPEND compile_definitions "NDEBUG")
#endif()

SET(RASC_CMAKE_EXE_LINKER_FLAGS "-g -O0 -mcpu=cortex-r52 -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -mthumb -T fsp.ld -Xlinker --gc-sections -Wl,-Map,${PROJECT_NAME}.map --specs=nano.specs --specs=rdimon.specs -L ${CMAKE_CURRENT_LIST_DIR}/../ -L ${CMAKE_CURRENT_LIST_DIR}/../script  -o ${CMAKE_CURRENT_LIST_DIR}/../build/CMakeFiles/${RASC_PROJECT_NAME}.elf.dir/${RASC_PROJECT_NAME}.elf")
#SET(RASC_ASM_FILES "${CMAKE_CURRENT_LIST_DIR}/../rzt_gen/*.asm")

add_compile_options(
        # CPU specific
        "-mcpu=cortex-r52"
        "-mthumb"
        "-mfloat-abi=hard"
        "-mfpu=neon-fp-armv8"
        # Non-CPU specific
#        "--specs=nano.specs"
        "-ffunction-sections"
        "-fdata-sections"
#        "-fstack-usage"
# From RZT but still not used in stm32
        "-fmessage-length=0"
        "-fsigned-char"
        # Other options
        "-g3"
#        "-c"
        "-Os"
        "-Wall"
# From RZT but still not used in stm32
#        "-Wunused"
#        "-Wuninitialized"
#        "-Wextra"
#        "-Wmissing-declarations"
#        "-Wconversion"
#        "-Wpointer-arith"
#        "-Wshadow"
#        "-Wlogical-op"
#        "-Waggregate-return"
#        "-Wfloat-equal"
)

add_compile_definitions(
#        USE_HAL_DRIVER
#        STM32F405xx
        _RENESAS_RZT_
        _RZT_CORE=CR52_0
        USE_TTERM)

include_directories(
        ../MESC_Common
        ../MESC_Common/Inc
        ../MESC_RTOS

        rzt/arm/CMSIS_5/CMSIS/Core_R/Include
        rzt/aws/amazon-freertos/freertos_kernel/include
        rzt/aws/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_tcp/include
        rzt/aws/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_tcp/source/portable/Compiler/GCC
        rzt/fsp/inc
        rzt/fsp/inc/api
        rzt/fsp/inc/instances
        rzt/fsp/src/rm_freertos_plus_tcp
        rzt/fsp/src/rm_freertos_port
        rzt_cfg/aws
        rzt_cfg/fsp_cfg
        rzt_cfg/fsp_cfg/bsp
        rzt_gen
        src/fsp
        src/CMSIS_RTOS_V2
        src)

#        Drivers/STM32F4xx_HAL_Driver/Inc
#        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
#        Middlewares/Third_Party/FreeRTOS/Source/include
#        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
#        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
#        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
#        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
#        Drivers/CMSIS/Device/ST/STM32F4xx/Include
#        Drivers/CMSIS/Include
#        Core/Inc
#        USB_DEVICE/App
#        USB_DEVICE/Target)

message(STATUS "CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")

file(GLOB_RECURSE PLATFORM_SOURCES *.c *.cpp)

add_executable(${PROJECT_NAME}.elf
        ${PLATFORM_SOURCES}
        ../MESC_Common/Src/fonts.c
#        ../MESC_Common/Src/MESC_BLDC.c
#        ../MESC_Common/Src/MESC_Comms.c
        ../MESC_Common/Src/MESCbat.c
#        ../MESC_Common/Src/MESCBLDC.c
        ../MESC_Common/Src/MESCcli.c
        ../MESC_Common/Src/MESCerror.c
        ../MESC_Common/Src/MESCflash.c
        ../MESC_Common/Src/MESCfnv.c
        ../MESC_Common/Src/MESCfoc.c
        ../MESC_Common/Src/MESCmotor.c
#        ../MESC_Common/Src/MESCmotor_state.c
        ../MESC_Common/Src/MESCposition.c
        ../MESC_Common/Src/MESCprofile.c
        ../MESC_Common/Src/MESCsin_lut.c
        ../MESC_Common/Src/MESCspeed.c
        ../MESC_Common/Src/MESCtemp.c
        ../MESC_Common/Src/MESCuart.c
        ../MESC_Common/Src/MESCui.c
#        ../MESC_Common/Src/MPU6050.c
#        ../MESC_Common/Src/Simple_coms.c
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
        ../MESC_RTOS/MESC/MESCinterface.c)

#target_include_directories(${PROJECT_NAME}.elf
#        PRIVATE
#        ${CMAKE_CURRENT_LIST_DIR}/MESC_Common/Inc
#        ${CMAKE_CURRENT_LIST_DIR}/MESC_Common)

set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINK_FLAGS
    "-mthumb \
    -mcpu=cortex-r52 \
	-T\"${CMAKE_CURRENT_LIST_DIR}/script/fsp.ld\" \
	-L${CMAKE_CURRENT_LIST_DIR} \
	-Wl,-Map=\"${PROJECT_NAME}.map\" \
    -Xlinker --gc-sections \
	--specs=nano.specs \
    --specs=rdimon.specs") # -static \ -Wl,--start-group -lc -lm -Wl,--end-group")

## The OBJCOPY utility will separate the EEPROM section from the main program
#add_custom_command(TARGET "${PROJECT_NAME}" POST_BUILD
#        COMMAND "${CMAKE_OBJCOPY}" ARGS -j .eeprom "$<TARGET_FILE:${PROJECT_NAME}>" -O ihex "eeprom.hex"
#        COMMAND "${CMAKE_OBJCOPY}" ARGS -R .eeprom "$<TARGET_FILE:${PROJECT_NAME}>" "program.elf"
#        BYPRODUCTS "eeprom.hex" "program.elf")

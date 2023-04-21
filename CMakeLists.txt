cmake_minimum_required(VERSION 3.16.4)

add_subdirectory(MESC_F405RG)

if (0)
include(MESC_RZT2M/cmake/gcc.cmake)

message(STATUS "RASC_EXE_PATH=${RASC_EXE_PATH}")
message(STATUS "CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")

#if (NOT "${RASC_EXE_PATH}" STREQUAL "")

project(MESC_RZT2M
        VERSION 1.0.0
        LANGUAGES C CXX ASM)

set(CMAKE_VERBOSE_MAKEFILE ON)

#source directories
file(GLOB_RECURSE Source_Files
        "${CMAKE_CURRENT_LIST_DIR}/MESC_Common/Src/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/rzt/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/rzt/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/rzt_gen/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/rzt_gen/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/src/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/MESC_RZT2M/src/*.cpp")

add_executable(${PROJECT_NAME}.elf
        ${Source_Files})

target_include_directories(${PROJECT_NAME}.elf
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/MESC_Common/Inc
        ${CMAKE_CURRENT_LIST_DIR}/MESC_Common)

#target_include_directories(${PROJECT_NAME}.elf
#    PUBLIC
#    ${MESC_Common_INCLUDE_DIR}
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/arm/CMSIS_5/CMSIS/Core_R/Include
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/aws/amazon-freertos/freertos_kernel/include
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/aws/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_tcp/include
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/fsp/inc
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/fsp/inc/api
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/fsp/inc/instances
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/fsp/src/rm_freertos_plus_tcp
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt/fsp/src/rm_freertos_port
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt_cfg/aws
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt_cfg/fsp_cfg
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt_cfg/fsp_cfg/bsp
#    ${CMAKE_CURRENT_LIST_DIR}/../rzt_gen
#    ${CMAKE_CURRENT_LIST_DIR}/../src
#    ${CMAKE_CURRENT_LIST_DIR}/../src/fsp
#)

#endif()
endif()

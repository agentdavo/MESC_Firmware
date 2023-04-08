# This file was automatically generated and updated by RASC

#source directories
file(GLOB_RECURSE Source_Files 
"${CMAKE_CURRENT_SOURCE_DIR}/rzt/*.c"
"${CMAKE_CURRENT_SOURCE_DIR}/rzt/*.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen/*.c"
"${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen/*.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")


SET(ALL_FILES ${Source_Files})

add_executable(${PROJECT_NAME}.elf ${ALL_FILES})

target_include_directories(${PROJECT_NAME}.elf
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/arm/CMSIS_5/CMSIS/Core_R/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/aws/amazon-freertos/freertos_kernel/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/aws/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_tcp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc/api
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc/instances
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/src/rm_freertos_plus_tcp
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/src/rm_freertos_port
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/aws
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/fsp_cfg
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/fsp_cfg/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


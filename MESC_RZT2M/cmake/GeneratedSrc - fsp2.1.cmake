# This file was automatically generated and updated by RASC

#source directories
file(GLOB_RECURSE Source_Files 
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)


SET(ALL_FILES ${Source_Files})

add_executable(${PROJECT_NAME}.elf
	${ALL_FILES}
)

target_compile_options(${PROJECT_NAME}.elf
                       PRIVATE
                       $<$<CONFIG:Debug>:${RASC_DEBUG_FLAGS}>
                       $<$<CONFIG:Release>:${RASC_RELEASE_FLAGS}>
                       $<$<CONFIG:MinSizeRel>:${RASC_MIN_SIZE_RELEASE_FLAGS}>
                       $<$<CONFIG:RelWithDebInfo>:${RASC_RELEASE_WITH_DEBUG_INFO}>)

target_compile_options(${PROJECT_NAME}.elf PRIVATE  $<$<COMPILE_LANGUAGE:C>:${RASC_CMAKE_C_FLAGS}>)
target_compile_options(${PROJECT_NAME}.elf PRIVATE  $<$<COMPILE_LANGUAGE:CXX>:${RASC_CMAKE_CXX_FLAGS}>)

target_link_options(${PROJECT_NAME}.elf PRIVATE $<$<LINK_LANGUAGE:C>:${RASC_CMAKE_EXE_LINKER_FLAGS}>)
target_link_options(${PROJECT_NAME}.elf PRIVATE $<$<LINK_LANGUAGE:CXX>:${RASC_CMAKE_EXE_LINKER_FLAGS}>)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${RASC_CMAKE_DEFINITIONS})

target_include_directories(${PROJECT_NAME}.elf
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/arm/CMSIS_5/CMSIS/Core_R/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/aws/amazon-freertos/freertos_kernel/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/aws/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_tcp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc/api
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/inc/instances
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/src/bsp/mcu/all/cr
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/src/rm_freertos_plus_tcp
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt/fsp/src/rm_freertos_port/cr
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/aws
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/fsp_cfg
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_cfg/fsp_cfg/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/rzt_gen
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(${PROJECT_NAME}.elf
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/script
)

target_link_libraries(${PROJECT_NAME}.elf
    PRIVATE
    
)

add_custom_target(obj_copy ALL
    COMMAND ${CMAKE_OBJCOPY} -O srec ${PROJECT_NAME}.elf ${PROJECT_NAME}.srec
    COMMENT "Creating S-record file in ${PROJECT_BINARY_DIR}"
)

add_dependencies(obj_copy ${PROJECT_NAME}.elf)

# Pre-build step: run RASC to generate project content if configuration.xml is changed
add_custom_command(
    OUTPUT
        configuration.xml.stamp
    COMMAND
        ${RASC_EXE_PATH}  -nosplash --launcher.suppressErrors --generate --devicefamily rzt --compiler GCC ${CMAKE_CURRENT_SOURCE_DIR}/configuration.xml
    COMMAND
        ${CMAKE_COMMAND} -E touch configuration.xml.stamp
    COMMENT
        "RASC pre-build to generate project content"
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/configuration.xml
)

add_custom_target(generate_content ALL
  DEPENDS configuration.xml.stamp
)

add_dependencies(${PROJECT_NAME}.elf generate_content)


# Post-build step: run RASC to generate the SmartBundle file
add_custom_command(
    TARGET
        ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND
        echo Running RASC post-build to generate Smart Bundle (.sbd) file
    COMMAND
        ${RASC_EXE_PATH} -nosplash --launcher.suppressErrors --gensmartbundle --devicefamily rzt --compiler GCC ${CMAKE_CURRENT_SOURCE_DIR}/configuration.xml ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf
    VERBATIM
)

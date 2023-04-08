# You can call cmake from command line by toolchain file argument   use `-DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake`

# Project minimum required CMake version
cmake_minimum_required(VERSION 3.16.4)

# Project configuration
project(MESC_RZT2M
	VERSION 1.0.0
	LANGUAGES C CXX ASM)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/GeneratedSrc.cmake)

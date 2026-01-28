# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Suppress git boundary warnings in containerized environments
set(ENV{GIT_DISCOVERY_ACROSS_FILESYSTEM} "1")

#set up ESP-IDF build environment
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ./components/bsp_extra
    ./apps/time_settings
    )

# Silence warnings from ESP-IDF system headers
# -Wno-unused-function: FreeRTOS atomic.h inline functions we don't use
# Note: M_PI warning is handled by include order in main.cpp (qmi8658.h before cmath)
add_compile_options(-Wno-unused-function)

project(main)
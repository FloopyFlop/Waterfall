cmake_minimum_required(VERSION 3.10)
project(px4_param_bridge C)

# Find PX4-Autopilot installation
# User should set PX4_DIR environment variable or pass it via cmake -DPX4_DIR=...
if(NOT DEFINED PX4_DIR)
    if(DEFINED ENV{PX4_DIR})
        set(PX4_DIR $ENV{PX4_DIR})
    else()
        message(FATAL_ERROR "PX4_DIR not set. Please set PX4_DIR environment variable or pass -DPX4_DIR=/path/to/PX4-Autopilot")
    endif()
endif()

message(STATUS "PX4_DIR: ${PX4_DIR}")

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3")

# Include directories from PX4
include_directories(
    ${PX4_DIR}/platforms/common/include
    ${PX4_DIR}/src/lib/parameters
    ${PX4_DIR}/build/px4_sitl_default/src/lib/parameters
)

# Link directories
link_directories(
    ${PX4_DIR}/build/px4_sitl_default/src/lib/parameters
    ${PX4_DIR}/build/px4_sitl_default
)

# Create shared library
add_library(px4_param_bridge SHARED px4_param_bridge.c)

# Link against PX4 parameters library
# The exact library name may vary depending on PX4 version
target_link_libraries(px4_param_bridge
    # These are potential library names - may need adjustment
    # parameters
    # px4_platform
)

# Install the library
install(TARGETS px4_param_bridge
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C flags: ${CMAKE_C_FLAGS}")
message(STATUS "Output library: libpx4_param_bridge.so (Linux) or libpx4_param_bridge.dylib (macOS)")

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_pio)

# Get the current build time
string(TIMESTAMP BUILD_TIME "%Y-%m-%d %H:%M:%S")

# Get the latest commit hash
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Pass the build time and commit hash as preprocessor definitions
add_definitions(-DBUILD_TIME="${BUILD_TIME}")
add_definitions(-DGIT_COMMIT_HASH="${GIT_COMMIT_HASH}")

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
target_sources(app PRIVATE src/usb.c)
target_sources(app PRIVATE src/main.c)

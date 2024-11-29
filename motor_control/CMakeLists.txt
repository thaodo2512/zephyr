# SPDX-License-Identifier: Apache-2.0
# Thao Do

if(CONFIG_MOTOR_CONTROL)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_MOTOR_CONTROL_PID pid.c)
target_sources_ifdef(CONFIG_MOTOR_CONTROL_ENCODER encoder.c)
target_sources_ifdef(CONFIG_MOTOR_CONTROL_USB_COMMUNICATION usd.c)

endif()

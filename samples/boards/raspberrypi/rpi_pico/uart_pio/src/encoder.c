/**
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


#include <stdio.h>
#include "pico/stdlib.h"
// #include "hardware/pio.h"
// #include "quadrature.pio.h"

#include "common.h"

LOG_MODULE_REGISTER(encoder, LOG_LEVEL_INF);

#define PULSES_PER_REVOLUTION (234)
#define MOTOR_DEFAULT_SPEED   (280)

// we wanna 2 GPIO to read the signal from encoder
// encoder parameters
// - motor 12V 280rpm
// - channel A and B have 11 pules per resolution
// - down ratio 21.3 : 1 -> that means we have 21.3 * 11 = 234.3 pulses per revolution
//                       -> 280rpm * 234.3 = 65604 pulses per minute
//                       -> 65604 / (60 * 1000) = 1.0934 pulses per ms




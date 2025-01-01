/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "common.h"

LOG_MODULE_REGISTER(pid);

// Initialize the PI controller
void pi_init(pi_controller *pi, float kp, float ki, float ts)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->ts = ts;
    pi->prev_error = 0.0f;
    pi->prev_output = 0.0f;

    LOG_INF("PI controller initialized with kp = %f, ki = %f, ts = %f ms", (double)kp, (double)ki, (double)ts);
}

// Update the PI controller using z-transform
float pi_cal(pi_controller *pi, float setpoint, float measuredValue)
{
    // Calculate current error
    float error = setpoint - measuredValue;

    // Proportional term
    float p_out = pi->kp * error;

    // Integral term (discrete accumulation using z-transform)
    float i_out = pi->prev_output + pi->ki * (pi->ts * 0.001f) * error; // 0.001f is to convert ms to s

    // Control signal
    float output = p_out + i_out;
    if (output >= 100.0f) {
        output = 100.0f;
    }

    if (output <= 0.0f) {
        output = 0.0f;
    }

    // Update the previous states
    pi->prev_error = error;
    pi->prev_output = output;

    return output;
}

float pi_pos_cal(pi_controller *pi, float setpoint, float measuredValue)
{
    // Calculate current error
    float error = setpoint - measuredValue;

    // Proportional term
    float p_out = pi->kp * error;

    // Integral term (discrete accumulation using z-transform)
    float i_out = pi->prev_output + pi->ki * (pi->ts * 0.001f) * error; // 0.001f is to convert ms to s

    // Control signal
    float output = p_out + i_out;
    if (output >= 100.0f) {
        output = 100.0f;
    }

    if (output <= -100.0f) {
        output = -100.0f;
    }

    // Update the previous states
    pi->prev_error = error;
    pi->prev_output = output;

    return output;
}

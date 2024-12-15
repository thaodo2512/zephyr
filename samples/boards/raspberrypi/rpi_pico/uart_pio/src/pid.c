/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <math.h>


#define PID_TS 0.0001  // sample time
#define PID_FILTER_COEFF 0.5 // filter coefficient
#define PID_K (2/PID_TS)

static float error[3] = {0, 0, 0};
static float control_signal[3] = {0, 0, 0};
static float a_coeff[3] = {0, 0, 0};
static float b_coeff[3] = {0, 0, 0};


float pid_cal(float kp, float ki, float kd)
{
    a_coeff[0] = PID_K * PID_K + PID_FILTER_COEFF * PID_K;
    a_coeff[1] = -2 * PID_K * PID_K;
    a_coeff[2] = PID_K * PID_K - PID_FILTER_COEFF * PID_K;

    b_coeff[0] = (PID_K * PID_K  + PID_K * PID_FILTER_COEFF) * kp  + \
                (PID_K + PID_FILTER_COEFF)* ki + PID_K * PID_K * kd * PID_FILTER_COEFF;
    b_coeff[1] = 2.0 * ki * PID_FILTER_COEFF - 2 * PID_K * PID_K * kp  - \
                2 * PID_K * PID_K * kd * PID_FILTER_COEFF;
    b_coeff[2] = (PID_K * PID_K * kp - PID_K  * ki) + ki * PID_FILTER_COEFF - \
                PID_K * kp * PID_FILTER_COEFF + PID_K * PID_K * kd * PID_FILTER_COEFF;

    return (-a_coeff[1]/a_coeff[0]) * control_signal[1] - (-a_coeff[2]/a_coeff[0])*control_signal[2] + \
            (b_coeff[0]/a_coeff[0]) * error[0] + (b_coeff[1]/a_coeff[0]) * error[1] + \
            (b_coeff[2]/a_coeff[0]) * error[2];
}

void pid_update_control_signal(float new_control_signal)
{
    control_signal[2] = control_signal[1];
    control_signal[1] = control_signal[0];
    control_signal[0] = new_control_signal;
}

void pid_update_error(float new_error)
{
    error[2] = error[1];
    error[1] = error[0];
    error[0] = new_error;
}

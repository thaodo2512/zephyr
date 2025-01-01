/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_DRIVE_COMMON_H
#define MOTOR_DRIVE_COMMON_H

#include <zephyr/kernel.h>
#include <stdint.h>

#define COMMUNICATION_MAX_SIZE 32
#define COMMUNICATION_VERSION 1

#define MOTOR_SAMPLING_TIME_MS 10

enum communication_cmd {
    COMMUNICATION_GET_SPEED = 0,
    COMMUNICATION_SET_SPEED = 1,
    COMMNUICATION_GET_POSITION = 2,
    COMMUNICATION_SET_POSITION = 3,
    COMMUNICATION_SET_P = 4,
    COMMUNICATION_SET_I = 5,
    COMMUNICATION_SET_D = 6,
};

struct common_header {
    uint8_t reserved;
    uint8_t ver;
    uint8_t cmd;
    uint8_t flags;
    uint8_t len;
    uint8_t crc;
} __packed;


struct common_message {
    struct common_header header;
    uint8_t payload[COMMUNICATION_MAX_SIZE - sizeof(struct common_header)];
} __packed;

/**
 * @brief Init communication thread
 *
 * @return int 0 if success, otherwise failed
 */
int communication_init(void);

typedef struct {
    float kp;         // Proportional gain
    float ki;         // Integral gain
    float ts;         // Sampling time
    float prev_error;  // Error at k-1
    float prev_output; // Control output at k-1
} pi_controller;

// Initialize the PI controller
void pi_init(pi_controller *pi, float kp, float ki, float ts);

/** calulcate pwm output */
float pi_cal(pi_controller *pi, float setpoint, float measuredValue);

float *get_set_point(void);

const struct device *get_encoder_device(void);

#endif

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

#define MOTOR_SAMPLING_TIME_MS 1

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

/** calulcate pwm output */
float pid_cal(float kp, float ki, float kd);

void pid_update_control_signal(float new_control_signal);

void pid_update_error(float new_error);

float *get_set_point(void);

const struct device *get_encoder_device(void);

#endif

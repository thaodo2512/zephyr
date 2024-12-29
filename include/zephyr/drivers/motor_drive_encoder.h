/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_DRIVE_ENCODER_H
#define MOTOR_DRIVE_ENCODER_H

/**
 * @brief Get the counter for scratch Y register
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param count The buffer cointains the counter value
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_encoder_get_count)(const struct device *dev, int32_t *count);

/**
 * @brief Reset the counter for scratch Y register
 *
 * @note: unsupported now
 * @param dev Pointer to the device structure for the driver instance.
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_encoder_reset_count)(const struct device *dev);

/**
 * @brief Get the position of the motor in degree
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param position The buffer cointains the position value
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_encoder_get_position)(const struct device *dev, int32_t *position);

/**
 * @brief Get the speed of mtor in RPM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param speed The buffer cointains the speed value
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_encoder_get_speed)(const struct device *dev, float *speed);

struct motor_drive_encoder_api {
    motor_drive_encoder_get_count get_count;
    motor_drive_encoder_reset_count reset_count;
    motor_drive_encoder_get_position get_position;
    motor_drive_encoder_get_speed get_speed;
};

/**
 * @brief Configuration param of encoder
 *
 * @param encoder_pulses_per_revolution The number of pulses per revolution
 * @param motor_gear_ratio The gear ratio of the motor
 *                         No_pulses_per_revolution = encoder_pulses_per_revolution * motor_gear_ratio
 * @param motor_default_speed The default speed of the motor when motor works at default voltage
 */
struct motor_drive_encoder_config {
    const uint32_t encoder_pulses_per_revolution;
    const uint32_t motor_gear_ratio;
    const uint32_t motor_default_speed;
    const uint32_t channel_a_pin;
    const uint32_t channel_b_pin;
};

#endif

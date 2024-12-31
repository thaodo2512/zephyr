/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/device.h>

#define MOTOR_DIRVE_DIRECTION_FORWARD  0
#define MOTOR_DIRVE_DIRECTION_BACKWARD 1

enum motor_drive_status {
	MOTOR_DRIVE_STATUS_OFF = 0,
	MOTOR_DRIVE_STATUS_ON,
	MOTOR_DRIVE_STATUS_ERROR,
};

struct motor_info {
	const char *label;
	uint32_t index;
	enum motor_drive_status status;
};

/**
 * @brief This API helps to turn on the motor which the specific direction
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param motor The motor index
 * @param dir The direction of the motor (0: forward, 1: backward)
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_on_t)(const struct device *dev, uint32_t motor, uint8_t dir);

/**
 * @brief This API helps to turn off the motor
 *        Force off by turning off 2 control GPIOs
 *
 * @param dev   Pointer to the device structure for the driver instance.
 * @param motor The motor index
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_off_t)(const struct device *dev, uint32_t motor);

/**
 * @brief This API helps to get the motor information
 *
 */
typedef int (*motor_drive_get_info_t)(const struct device *dev, uint32_t motor,
				      const struct motor_info **info);

/** @brief Set the output voltage for the motor
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param motor The motor index
 * @param value The speed value
 * @retval int 0 if successful, negative errno code on failure.
 */
typedef int (*motor_drive_set_output_voltage_t)(const struct device *dev, uint32_t motor, float value);

struct motor_driver_api {
	motor_drive_on_t on;
	motor_drive_off_t off;
	motor_drive_set_output_voltage_t set_voltage;
	motor_drive_get_info_t get_info;
};

#endif /* MOTOR_DRIVE_H */

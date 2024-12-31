/*
 * Copyright (c) 2023 Yonatan Schachter
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/drivers/motor_drive.h>
#include <zephyr/drivers/motor_drive_encoder.h>
#include "common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Define the build time and commit hash if not already defined
#ifndef BUILD_TIME
#define BUILD_TIME "Unknown"
#endif

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "Unknown"
#endif

#define MOTOR_SAMPLING_TIME_ZEPHYR_MS K_SECONDS(MOTOR_SAMPLING_TIME_MS)

static void timer_expiry_fnc(struct k_timer *timer);
K_TIMER_DEFINE(pid_timer, timer_expiry_fnc, NULL);
K_SEM_DEFINE(pid_sem, 0, 1);
static float set_point = 0.0f;

static void timer_expiry_fnc(struct k_timer *timer)
{
	k_sem_give(&pid_sem);
}

float *get_set_point(void)
{
	return &set_point;
}

const struct device *get_encoder_device(void)
{
	return DEVICE_DT_GET(DT_NODELABEL(motor_drive_encoder0));
}


int main(void)
{
	LOG_INF("Booting to raspberry pi pico - bai tap lon 2024");
    LOG_INF("Build time: %s", BUILD_TIME);
    LOG_INF("Commit hash: %s", GIT_COMMIT_HASH);

	int rc;
	const struct device *motor_drive = DEVICE_DT_GET(DT_NODELABEL(motor_drive0));
	const struct device *encoder = DEVICE_DT_GET(DT_NODELABEL(motor_drive_encoder0));
	struct motor_driver_api *motor_driver_api;
	struct motor_drive_encoder_api *encoder_api;

	if (!device_is_ready(motor_drive)) {
		LOG_ERR("Motor drive device %s is not ready", motor_drive->name);
		return -ENODEV;
	}

	if (!device_is_ready(encoder)) {
		LOG_ERR("Encoder device %s is not ready", encoder->name);
		return -ENODEV;
	}

	motor_driver_api = (struct motor_driver_api *)motor_drive->api;
	encoder_api = (struct motor_drive_encoder_api *)encoder->api;

	rc = communication_init();
	if (rc) {
		LOG_ERR("failed to setup communication - rc = %d", rc);
		return rc;
	}

	rc = motor_driver_api->off(motor_drive, 0);
	if (rc) {
		LOG_ERR("failed to turn off motor - rc = %d", rc);
	}

	rc = motor_driver_api->set_voltage(motor_drive, 0, 0);
	if (rc) {
		LOG_ERR("failed to set speed - rc = %d", rc);
	}

	k_timer_start(&pid_timer, MOTOR_SAMPLING_TIME_ZEPHYR_MS, MOTOR_SAMPLING_TIME_ZEPHYR_MS);
	// float speed = 0.0f;
	// int position = 0;
	// int cnt = 0;
	// int pre_cnt = 0;

	while (1) {
		k_sem_take(&pid_sem, K_FOREVER);
		// rc = encoder_api->get_speed(encoder, &speed);
		// if (rc) {
		// 	LOG_ERR("failed to get encoder count - rc = %d", rc);
		// } else {
		// 	LOG_INF("current speed = %f", (double)speed);
		// }

		// rc = encoder_api->get_position(encoder, &position);
		// if (!rc) {
		// 	LOG_INF("current position = %d", position);
		// }

		// rc = encoder_api->get_count(encoder, &cnt);
		// if (!rc) {
		// 	LOG_INF("current count = %d, delta = %d", cnt, cnt -pre_cnt);
		// 	pre_cnt = cnt;
		// }

		rc = motor_driver_api->on(motor_drive, 0, MOTOR_DIRVE_DIRECTION_FORWARD);
		if (rc) {
			LOG_ERR("failed to turn on motor - rc = %d", rc);
			continue;
		}

		rc = motor_driver_api->set_voltage(motor_drive, 0, 10.0f);
		if (rc) {
			LOG_ERR("failed to set speed - rc = %d", rc);
		}

	}

	return 0;
}

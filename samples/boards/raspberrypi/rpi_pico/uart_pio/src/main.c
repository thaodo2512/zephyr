/*
 * Copyright (c) 2023 Yonatan Schachter
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/drivers/motor_drive.h>
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


int main(void)
{
	LOG_INF("Booting to raspberry pi pico - bai tap lon 2024");
    LOG_INF("Build time: %s", BUILD_TIME);
    LOG_INF("Commit hash: %s", GIT_COMMIT_HASH);

	int rc;
	const struct device *motor_drive = DEVICE_DT_GET(DT_NODELABEL(motor_drive0));
	struct motor_driver_api *motor_driver_api;

	if (!device_is_ready(motor_drive)) {
		LOG_ERR("Motor drive device %s is not ready", motor_drive->name);
		return -ENODEV;
	}

	motor_driver_api = (struct motor_driver_api *)motor_drive->api;

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

	while (1) {
		k_sem_take(&pid_sem, K_FOREVER);
		LOG_INF("pid run with set point %.4f", (double)*get_set_point());
		// printf("pid run with set point %.4f\n", (double)*get_set_point());

		rc = motor_driver_api->set_voltage(motor_drive, 0, 5);
		if (rc) {
			LOG_ERR("failed to set speed - rc = %d", rc);
		}

	}

	return 0;
}

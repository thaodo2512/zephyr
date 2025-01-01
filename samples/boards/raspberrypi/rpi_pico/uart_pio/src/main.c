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

// #define MOTOR_SAMPLING_TIME_ZEPHYR_MS K_MSEC(MOTOR_SAMPLING_TIME_MS)
#define MOTOR_SAMPLING_TIME_ZEPHYR_MS K_SECONDS(10)

static void timer_expiry_fnc(struct k_timer *timer);
K_TIMER_DEFINE(pid_timer, timer_expiry_fnc, NULL);
K_SEM_DEFINE(pid_sem, 0, 1);
static float set_point = 0.0f;
static pi_controller controller;

static void timer_expiry_fnc(struct k_timer *timer)
{
	k_sem_give(&pid_sem);
}

static void timer_get_control_signal(struct k_timer *timer)
{
	const struct motor_drive_encoder_api *encoder_api = get_encoder_device()->api;
	float speed = 0.0f;

	encoder_api->get_speed(get_encoder_device(), &speed);

	// LOG_INF("current speed: %f", (double)speed);
	printk("%f\n", (double)speed);
}
K_TIMER_DEFINE(get_signal_timer, timer_get_control_signal, NULL);

float *get_set_point(void)
{
	return &set_point;
}

const struct device *get_encoder_device(void)
{
	return DEVICE_DT_GET(DT_NODELABEL(motor_drive_encoder0));
}

pi_controller *get_pi_controller(void)
{
	return &controller;
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
	float volt = 100.0f;
	float speed = 0.0f;

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

	k_timer_start(&get_signal_timer, K_SECONDS(2), K_SECONDS(2));
	k_timer_start(&pid_timer, MOTOR_SAMPLING_TIME_ZEPHYR_MS, MOTOR_SAMPLING_TIME_ZEPHYR_MS);
	pi_init(get_pi_controller(), 2, 0.1, MOTOR_SAMPLING_TIME_MS);

	rc = motor_driver_api->on(motor_drive, 0, MOTOR_DIRVE_DIRECTION_FORWARD);
	if (rc) {
		LOG_ERR("failed to turn on motor - rc = %d", rc);
		return rc;
	}

	// speed range 70 rpm to 200 rpm

	while (1) {
		k_sem_take(&pid_sem, K_FOREVER);

		rc = encoder_api->get_speed(encoder, &speed);
		if (rc) {
			LOG_ERR("failed to get speed - rc = %d", rc);
			continue;
		}

		// volt = pi_cal(&controller, 70.0f, speed);
		// volt = 2.0f;
		volt -= 5;
		if (volt < 60.0f) {
			volt = 100.0f;
		}

		// static int test = 0;

		// if (test) {
		// 	rc = motor_driver_api->set_voltage(motor_drive, 0, 2);
		// 	if (rc) {
		// 		LOG_ERR("failed to set speed - rc = %d", rc);
		// 	}
		// } else {
		// 	rc = motor_driver_api->set_voltage(motor_drive, 0, 12);
		// 	if (rc) {
		// 		LOG_ERR("failed to set speed - rc = %d", rc);
		// 	}
		// }

		// test = !test;
		printk("set voltage: %f\n", (double)volt);
		rc = motor_driver_api->set_voltage(motor_drive, 0, volt);
		if (rc) {
			LOG_ERR("failed to set speed - rc = %d", rc);
		}
	}

	return 0;
}

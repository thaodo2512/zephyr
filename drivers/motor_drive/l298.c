/**
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/drivers/motor_drive.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_driver);

#define DT_DRV_COMPAT thao_do_motor_drive

struct motor_drive_config {
	int num_motors;
	const struct pwm_dt_spec motor;
	const struct gpio_dt_spec pin_io_0;
	const struct gpio_dt_spec pin_io_1;
};

static __unused int motor_pwm_set_output(const struct device *dev, uint32_t motor, uint32_t percent)
{
	struct motor_drive_config *config;
	uint32_t period;

	if (!dev) {
		return -EINVAL;
	}

	config = (struct motor_drive_config *)dev->config;
	if (!config) {
		assert_print("%s: config should be not NULL\n", __func__);
	}

	if (motor > config->num_motors) {
		return -EINVAL;
	}

	const struct pwm_dt_spec *dt_motor = &config->motor;

	// covert percent to pulse_nsec
	period = (dt_motor->period * percent) / 100;

	return pwm_set_pulse_dt(dt_motor, period);
}

static int motor_get_inf(const struct device *dev, uint32_t motor, const struct motor_info **info)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(motor);
	ARG_UNUSED(info);

	return 0;
}

static int motor_set_voltage(const struct device *dev, uint32_t motor, float value)
{
	// convert value (0 volt - 12 volt) to percent (0 - 100)
	float percent = (value / 12.0f) * 100.0f;

	LOG_DBG("set out voltage = %u %%", (uint32_t)percent);

	return motor_pwm_set_output(dev, motor, (uint32_t)percent);
}

static int motor_drive_on(const struct device *dev, uint32_t motor, uint8_t dir)
{
	struct motor_drive_config *cfg;
	int rc;

	if (!dev) {
		return -EINVAL;
	}

	cfg = (struct motor_drive_config *)dev->config;
	if (!cfg) {
		assert_print("%s: config should be not NULL\n", __func__);
	}

	if (motor > cfg->num_motors) {
		return -EINVAL;
	}

	if (dir == MOTOR_DIRVE_DIRECTION_FORWARD) {
		rc = gpio_pin_set_dt(&cfg->pin_io_0, 1);
		if (rc) {
			LOG_DBG("cannot set pin 0 to 1");
			return rc;
		}
		rc = gpio_pin_set_dt(&cfg->pin_io_1, 0);
		if (rc) {
			LOG_DBG("cannot set pin 1 to 0");
			return rc;
		}
	} else {
		rc = gpio_pin_set_dt(&cfg->pin_io_0, 0);
		if (rc) {
			LOG_DBG("cannot set pin 0 to 0");
			return rc;
		}
		rc = gpio_pin_set_dt(&cfg->pin_io_1, 1);
		if (rc) {
			LOG_DBG("cannot set pin 1 to 0");
			return rc;
		}
	}

	return 0;
}

static int motor_drive_off(const struct device *dev, uint32_t motor)
{
	struct motor_drive_config *cfg;
	int rc = 0;

	if (!dev) {
		return -EINVAL;
	}

	cfg = (struct motor_drive_config *)dev->config;
	if (!cfg) {
		assert_print("%s: config should be not NULL\n", __func__);
	}

	if (motor > cfg->num_motors) {
		return -EINVAL;
	}

	rc = gpio_pin_set_dt(&cfg->pin_io_0, 0);
	if (rc) {
		LOG_ERR("%s: cannot set pin 0 to 0", __func__);
		return rc;
	}
	rc = gpio_pin_set_dt(&cfg->pin_io_1, 0);
	if (rc) {
		LOG_ERR("%s: cannot set pin 1 to 0", __func__);
		return rc;
	}

	return 0;
}

static int motor_pwm_init(const struct device *dev)
{
	struct motor_drive_config *config = (struct motor_drive_config *)dev->config;
	int rc;

	LOG_INF("init");

	if (!config->num_motors) {
		LOG_ERR("%s: no motors found (DT child nodes missing)", dev->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->pin_io_0) || !gpio_is_ready_dt(&config->pin_io_1)) {
		LOG_ERR("%s: gpio device not ready", dev->name);
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&config->pin_io_0, GPIO_OUTPUT_LOW);
	if (rc) {
		LOG_ERR("%s: unable to configure pin 0", dev->name);
		return rc;
	}
	rc = gpio_pin_configure_dt(&config->pin_io_1, GPIO_OUTPUT_LOW);
	if (rc) {
		LOG_ERR("%s: unable to configure pin 1", dev->name);
		return rc;
	}

	return 0;
}

static struct motor_driver_api motor_driver_api = {
	.on = motor_drive_on,
	.off = motor_drive_off,
	.set_voltage = motor_set_voltage,
	.get_info = motor_get_inf,
};

#if IS_ENABLED(CONFIG_PWM)
#define MOTOR_DRIVE_GET_PWM_DEVICE(inst) PWM_DT_SPEC_GET(DT_DRV_INST(inst))
#else
#define MOTOR_DRIVE_GET_PWM_DEVICE(inst)                                                           \
	{                                                                                          \
		0                                                                                  \
	}
#endif // CONFIG_PWM

#if IS_ENABLED(CONFIG_GPIO)

#define MOTOR_DRIVE_GET_GPIO(inst, pro_name) GPIO_DT_SPEC_GET(DT_INST_CHILD(inst, pro_name), gpios)

#else

#define MOTOR_DRIVE_GET_GPIO(inst, pro_name)                                                       \
	{                                                                                          \
		0                                                                                  \
	}

#endif // CONFIG_GPIO

#define MOTOR_PWM_DEVICE(inst)                                                                     \
	static struct motor_drive_config motor_drive_config##inst = {                              \
		.num_motors = 1,                                                                   \
		.motor = MOTOR_DRIVE_GET_PWM_DEVICE(inst),                                         \
		.pin_io_0 = MOTOR_DRIVE_GET_GPIO(inst, gpio_ctrl0),                                \
		.pin_io_1 = MOTOR_DRIVE_GET_GPIO(inst, gpio_ctrl1),                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, motor_pwm_init, NULL, NULL, &motor_drive_config##inst,         \
			      POST_KERNEL, CONFIG_MOTOR_DRIVE_DRIVER_INIT_PRIORITY, &motor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_PWM_DEVICE)

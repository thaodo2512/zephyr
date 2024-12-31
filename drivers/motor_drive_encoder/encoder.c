/**
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <zephyr/drivers/motor_drive_encoder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rpi_pico_encoder);

#define DT_DRV_COMPAT thao_do_pico_encoder

// we wanna 2 GPIO to read the signal from encoder
// encoder parameters
// - motor 12V 280rpm
// - channel A and B have 11 pules per resolution
// - down ratio 21.3 : 1 -> that means we have 21.3 * 11 = 234.3 pulses per revolution
//                       -> 280rpm * 234.3 = 65604 pulses per minute
//                       -> 65604 / (60 * 1000) = 1.0934 pulses per ms

struct motor_drive_encoder_data {
	struct device *encoder;
	const struct device *piodev;
	PIO pio;
	size_t sm;
	struct k_timer timer;
	uint32_t sampling_time;
	float speed;    // rpm
	float position; // degree
};

#define quadrature_encoder_wrap_target 15
#define quadrature_encoder_wrap        23
#define quadrature_encoder_pio_version 0

static const uint16_t quadrature_encoder_program_instructions[] = {
	0x000f, //  0: jmp    15
	0x000e, //  1: jmp    14
	0x0015, //  2: jmp    21
	0x000f, //  3: jmp    15
	0x0015, //  4: jmp    21
	0x000f, //  5: jmp    15
	0x000f, //  6: jmp    15
	0x000e, //  7: jmp    14
	0x000e, //  8: jmp    14
	0x000f, //  9: jmp    15
	0x000f, // 10: jmp    15
	0x0015, // 11: jmp    21
	0x000f, // 12: jmp    15
	0x0015, // 13: jmp    21
	0x008f, // 14: jmp    y--, 15
		//     .wrap_target
	0xa0c2, // 15: mov    isr, y
	0x8000, // 16: push   noblock
	0x60c2, // 17: out    isr, 2
	0x4002, // 18: in     pins, 2
	0xa0e6, // 19: mov    osr, isr
	0xa0a6, // 20: mov    pc, isr
	0xa04a, // 21: mov    y, !y
	0x0097, // 22: jmp    y--, 23
	0xa04a, // 23: mov    y, !y
		//     .wrap
};

static const struct pio_program quadrature_encoder_program = {
	.instructions = quadrature_encoder_program_instructions,
	.length = 24,
	.origin = 0,
	.pio_version = quadrature_encoder_pio_version,
};

static inline pio_sm_config quadrature_encoder_program_get_default_config(uint offset)
{
	pio_sm_config config = pio_get_default_sm_config();
	// sm_config_set_wrap(&config, offset + RPI_PICO_PIO_GET_WRAP_TARGET(quad_encoder),
	// 		   offset + RPI_PICO_PIO_GET_WRAP(quad_encoder));
	sm_config_set_wrap(&config, offset + quadrature_encoder_wrap_target,
			   offset + quadrature_encoder_wrap);
	return config;
}

// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint channel_a, uint channel_b,
						   int max_step_rate)
{
	uint32_t offset;
	pio_sm_config sm_config;

	if (channel_b != (channel_a + 1)) {
		LOG_ERR("Channel B must be next to channel A");
		return;
	}

	if (!pio_can_add_program(pio, &quadrature_encoder_program)) {
		return;
	}

	pio_sm_set_consecutive_pindirs(pio, sm, channel_a, 2, false);
	pio_gpio_init(pio, channel_a);
	pio_gpio_init(pio, channel_b);
	gpio_pull_up(channel_a);
	gpio_pull_up(channel_b);

	offset = pio_add_program(pio, &quadrature_encoder_program);
	sm_config = quadrature_encoder_program_get_default_config(0);

	sm_config_set_in_pins(&sm_config, channel_a); // for WAIT, IN
	sm_config_set_jmp_pin(&sm_config, channel_a); // for JMP
	// shift to left, autopull disabled
	sm_config_set_in_shift(&sm_config, false, false, 32);
	// don't join FIFO's
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_NONE);
	// passing "0" as the sample frequency,
	if (max_step_rate == 0) {
		sm_config_set_clkdiv(&sm_config, 1.0);
	} else {
		// one state machine loop takes at most 10 cycles
		float div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
		sm_config_set_clkdiv(&sm_config, div);
	}
	pio_sm_init(pio, sm, 0, &sm_config);
	pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm)
{
	static int32_t ret;
	int n;
	// if the FIFO has N entries, we fetch them to drain the FIFO,
	// plus one entry which will be guaranteed to not be stale
	if (pio_sm_is_rx_fifo_empty(pio, sm)) {
		LOG_DBG("pio rx empty");
		return ret;
	}
	n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
	while (n > 0) {
		ret = pio_sm_get_blocking(pio, sm);
		n--;
	}
	return ret;
}

static int encoder_get_count(const struct device *dev, int32_t *count)
{
	struct motor_drive_encoder_data *data;

	if (!device_is_ready(dev) || !count) {
		LOG_ERR("invalid param for %s", __func__);
		return -EINVAL;
	}

	data = dev->data;

	*count = quadrature_encoder_get_count(data->pio, data->sm);

	return 0;
}

static int encoder_get_position(const struct device *dev, float *position)
{
	struct motor_drive_encoder_data *data;

	if (!device_is_ready(dev) || !position) {
		LOG_ERR("invalid param for %s", __func__);
		return -EINVAL;
	}

	data = dev->data;

	*position = data->position;
	asm volatile ("" ::: "memory");

	return 0;
}

static int encoder_reset_counter(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_INF("%s: unsupported", __func__);

	return 0;
}

static int encoder_get_speed(const struct device *dev, float *speed)
{
	struct motor_drive_encoder_data *data;

	if (!device_is_ready(dev) || !speed) {
		LOG_ERR("invalid param for %s", __func__);
		return -EINVAL;
	}

	data = dev->data;
	*speed = data->speed;
	asm volatile ("" ::: "memory");

	return 0;
}

 void encoder_timer_handler(struct k_timer *dummy)
{
	struct motor_drive_encoder_data *data =
		CONTAINER_OF(dummy, struct motor_drive_encoder_data, timer);
	const struct motor_drive_encoder_api *api = data->encoder->api;
	const struct motor_drive_encoder_config *config = data->encoder->config;
	static int cnt = 0;
	static int pre_cnt = 0;
	int rc;
	float degree;

	rc = api->get_count(data->encoder, &cnt);
	if (rc) {
		LOG_ERR("failed to get encoder count - rc = %d", rc);
		return;
	}

	// convert to positive value
	if (cnt < 0) {
		cnt = -cnt;
	}

	// overlap case
	if (cnt < pre_cnt) {
		degree = (cnt + (INT_MAX - pre_cnt)) * 360.0f / (config->encoder_pulses_per_revolution * config->motor_gear_ratio);
	} else {
		degree = (cnt - pre_cnt) * 360.0f / (config->encoder_pulses_per_revolution * config->motor_gear_ratio);
	}

	// calculate speed
	data->speed = (degree / data->sampling_time) * 166666.67f;
	asm volatile ("" ::: "memory");

	// calculate position
	data->position += ((cnt - pre_cnt) * 360.0f);
	asm volatile ("" ::: "memory");

	pre_cnt = cnt;

	return;
}

static int pico_pio_encoder_init(const struct device *dev)
{
	const struct motor_drive_encoder_config *config = dev->config;
	struct motor_drive_encoder_data *data = dev->data;
	int rc;
	PIO pio;
	size_t sm;
	uint32_t channel_a_pin = config->channel_a_pin;
	uint32_t channel_b_pin = config->channel_b_pin;

	pio = pio_rpi_pico_get_pio(data->piodev);
	rc = pio_rpi_pico_allocate_sm(data->piodev, &sm);
	data->encoder = (struct device *)dev;
	data->pio = pio;
	data->sm = sm;

	LOG_INF("init encoder with pin %u and %u", channel_a_pin, channel_b_pin);
	LOG_INF("state machine number = %d", sm);
	LOG_INF("encoder pulses per revolution = %d", config->encoder_pulses_per_revolution);
	LOG_INF("encoder motor gear ratio = %d", config->motor_gear_ratio);
	LOG_INF("sampling time = %d ms", data->sampling_time);
	LOG_INF("factor : %f", (double)(config->encoder_pulses_per_revolution * config->motor_gear_ratio));

	if (rc < 0) {
		LOG_ERR("fail to allocate PIO SM for encoder");
		return rc;
	}
	quadrature_encoder_program_init(pio, sm, channel_a_pin, channel_b_pin, 1000);
	k_timer_init(&data->timer, encoder_timer_handler, NULL);
	k_timer_start(&data->timer, K_MSEC(data->sampling_time), K_MSEC(data->sampling_time));

	return 0;
}

struct motor_drive_encoder_api encoder_api = {.get_count = encoder_get_count,
					      .reset_count = encoder_reset_counter,
					      .get_position = encoder_get_position,
					      .get_speed = encoder_get_speed};

#define MOTOR_DRIVE_ENCODE_INIT(inst)                                                              \
	struct motor_drive_encoder_data encoder_data##inst = {                                     \
		.piodev = DEVICE_DT_GET(DT_CHOSEN(zephyr_encoder_pio)), \
		.sampling_time = DT_INST_PROP(inst, sampling_time)};                           \
	struct motor_drive_encoder_config encoder_config##inst = {                                 \
		.encoder_pulses_per_revolution = DT_INST_PROP(inst, pulses_per_revolution),        \
		.motor_gear_ratio = DT_INST_PROP(inst, motor_gear_ratio),                          \
		.motor_default_speed = DT_INST_PROP(inst, motor_default_speed),                    \
		.channel_a_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, channel_a, 0), \
		.channel_b_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, channel_b, 0), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pico_pio_encoder_init, NULL, &encoder_data##inst,              \
			      &encoder_config##inst, POST_KERNEL, 10, &encoder_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DRIVE_ENCODE_INIT)

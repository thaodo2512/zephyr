/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/motor_drive_encoder.h>

#include "common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(communicate);

#define COMMUNICATION_SET_RESPONSE(msg, command, payload_len)                                      \
	do {                                                                                       \
		(msg)->header.ver = COMMUNICATION_VERSION;                                         \
		(msg)->header.cmd = command;                                                       \
		(msg)->header.len = payload_len + sizeof(struct common_header);                    \
		(msg)->header.crc = 0;                                                             \
	} while (0)

#define COMMUNICATION_CONVERT_FLOAT_TO_BYTE_ARRAY(f, b)                                            \
	do {                                                                                       \
		memcpy(b, &f, sizeof(float));                                                      \
	} while (0)

#define COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(b, f)                                            \
	do {                                                                                       \
		memcpy(&(f), (b), sizeof(float));                                                  \
	} while (0)

struct communicate_context {
	struct k_thread communicate_thread;
	K_KERNEL_STACK_MEMBER(stack, 1024);
	struct k_sem msg_in;
};

struct uart_message {
	struct common_message *msg;
	int index;
	atomic_t done;
	atomic_t busy;
	atomic_t error;
};

static void timer_wa_handler(struct k_timer *timer);
K_TIMER_DEFINE(timer_wa, timer_wa_handler, NULL);

static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
static struct communicate_context uart_ctx;
static struct common_message recevice_msg_ = {0};
static struct common_message send_msg_ = {0};
static struct uart_message uart_recv_msg = {
	.msg = &recevice_msg_,
	.index = 0,
	.done = ATOMIC_INIT(0),
	.busy = ATOMIC_INIT(0),
	.error = ATOMIC_INIT(0),
};
static struct uart_message uart_send_msg = {
	.msg = &send_msg_,
	.index = 0,
	.done = ATOMIC_INIT(0),
	.busy = ATOMIC_INIT(0),
	.error = ATOMIC_INIT(0),
};

static inline struct uart_message *get_recv_msg(void)
{
	return &uart_recv_msg;
}

static inline struct uart_message *get_send_msg(void)
{
	return &uart_send_msg;
}

// after a period of time, if there is no new data, reset receiver buffer
static void timer_wa_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (atomic_get(&get_recv_msg()->busy) == 1 || atomic_get(&get_recv_msg()->done) == 1) {
		// buffer in use, do nothing
		return;
	} else {
		LOG_DBG("%s: reset receiver buffer", __func__);
		memset(get_recv_msg()->msg, 0, sizeof(struct common_message));
		atomic_set(&get_recv_msg()->busy, 0);
		atomic_set(&get_recv_msg()->done, 0);
		atomic_set(&get_recv_msg()->error, 0);
		get_recv_msg()->index = 0;
	}
}

static void uart_rx_handler(const struct device *uart_dev, void *user_data)
{
	struct communicate_context *usr_ctx = (struct communicate_context *)user_data;
	uint8_t data;
	int rc;

	if (uart_irq_rx_ready(uart_dev) != 1) {
		return;
	}

	if (atomic_get(&get_recv_msg()->busy) == 1 || atomic_get(&get_recv_msg()->done) == 1) {
		return;
	}

	rc = uart_fifo_read(uart_dev, &data, sizeof(data));
	if (!rc) {
		// no more data
		return;
	}

	LOG_DBG("uart recv: 0x%02x - index = %d", data, get_recv_msg()->index);

	*((char *)get_recv_msg()->msg + get_recv_msg()->index) = data;

	if (get_recv_msg()->index == 0) {
		// after 0.5 second, if there is no new data, reset receiver buffer
		k_timer_start(&timer_wa, K_MSEC(500), K_NO_WAIT);
	}
	get_recv_msg()->index++;
	if (get_recv_msg()->index >= sizeof(struct common_message)) {
		LOG_ERR("MCU-uart message is too long");
		atomic_set(&get_recv_msg()->error, 1);
		return;
	}

	// check if message is done
	if (get_recv_msg()->msg->header.len == get_recv_msg()->index) {
		atomic_set(&get_recv_msg()->done, 1);
		k_timer_stop(&timer_wa);
		k_sem_give(&usr_ctx->msg_in);
	}
}

static void uart_tx_handler(const struct device *uart_dev, void *user_data)
{
	int rc;
	static int priv_cnt = 0;

	if (uart_irq_tx_ready(uart_dev) <= 0) {
		return;
	}

	if (atomic_get(&get_send_msg()->error) == 1 || atomic_get(&get_send_msg()->done) == 0) {
		// note: in some cases, tx_read is still true but interrupt from another source
		LOG_DBG("error or not done");
		return;
	}

	if (get_send_msg()->index <= 0 && atomic_get(&get_send_msg()->done) == 1) {
		// complete to send message
		atomic_set(&get_send_msg()->done, 0);
		atomic_set(&get_send_msg()->busy, 0);
		// printk("total send %d bytes\n", priv_cnt);
		priv_cnt = 0;

		// release recv message buffer
		memset(get_recv_msg()->msg, 0, sizeof(struct common_message));
		atomic_set(&get_recv_msg()->busy, 0);
		atomic_set(&get_recv_msg()->done, 0);
		get_recv_msg()->index = 0;

		LOG_DBG("disable tx again");
		uart_irq_tx_disable(uart_dev);
		return;
	}

	// set buffer to busy state
	atomic_set(&get_send_msg()->busy, 1);
	rc = uart_fifo_fill(uart_dev, (char *)get_send_msg()->msg + priv_cnt, 1);
	if (rc <= 0) {
		LOG_WRN("could not send message (uart)");
		return;
	}

	priv_cnt++;
	get_send_msg()->index--;
}

/**
 * @brief UART interrupt callback
 *
 * @param uart_dev  uart device
 * @param user_data users's data
 */
static void uart_isr_cb(const struct device *uart_dev, void *user_data)
{
	while (uart_irq_is_pending(uart_dev) > 0 && uart_irq_update(uart_dev) > 0) {
		uart_rx_handler(uart_dev, user_data);
		uart_tx_handler(uart_dev, user_data);
	}
}

static int message_handler(struct common_message *msg, uint8_t len)
{
	struct common_message *recv_msg = (struct common_message *)msg;
	int rc = -ENODATA;
	float speed;
	float position;
	float p_cof;
	float i_cof;
	float d_cof;

	if (recv_msg->header.ver != COMMUNICATION_VERSION) {
		LOG_ERR("invalid version");
		return -EINVAL;
	}

	// check crc
	// uint8_t crc = crc8_ccitt(0, msg, len);
	// if (crc != recv_msg->header.crc) {
	// 	LOG_ERR("invalid crc");
	// 	return -EINVAL;
	// }

	((struct motor_drive_encoder_api *)get_encoder_device()->api)
		->get_speed(get_encoder_device(), &speed);
	((struct motor_drive_encoder_api *)get_encoder_device()->api)
		->get_position(get_encoder_device(), &position);
	// printk("command %d\n", recv_msg->header.cmd);

	switch (recv_msg->header.cmd) {
	case COMMUNICATION_GET_SPEED:
		COMMUNICATION_SET_RESPONSE(get_send_msg()->msg, COMMUNICATION_GET_SPEED,
					   sizeof(float));
		memcpy(get_send_msg()->msg->payload, &speed, sizeof(float));
		get_send_msg()->index = get_send_msg()->msg->header.len;
		LOG_DBG("get speed");
		rc = 0;
		break;
	case COMMUNICATION_SET_SPEED:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, speed);
		*get_set_point() = speed;
		*is_speed_control() = true;
		LOG_INF("set speed %f", (double)speed);
		break;
	case COMMNUICATION_GET_POSITION:
		COMMUNICATION_SET_RESPONSE(get_send_msg()->msg, COMMNUICATION_GET_POSITION,
					   sizeof(float));
		memcpy(get_send_msg()->msg->payload, &position, sizeof(float));
		get_send_msg()->index = get_send_msg()->msg->header.len;
		LOG_DBG("get position");
		rc = 0;
		break;
	case COMMUNICATION_SET_POSITION:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, position);
		*get_set_point() = position;
		*is_speed_control() = false;
		LOG_INF("set position %f", (double)position);
		break;
	case COMMUNICATION_SET_P:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, p_cof);
		get_pi_controller()->kp = p_cof;
		LOG_INF("set P %f", (double)p_cof);
		break;
	case COMMUNICATION_SET_I:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, i_cof);
		get_pi_controller()->ki = i_cof;
		LOG_INF("set I %f", (double)i_cof);
		break;
	case COMMUNICATION_SET_D:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, d_cof);
		get_pi_controller()->kd = d_cof;
		LOG_INF("set D %f", (double)d_cof);
		break;
	case COMMUNICATION_GET_SPEED_POSITION:
		COMMUNICATION_SET_RESPONSE(get_send_msg()->msg, COMMUNICATION_GET_SPEED_POSITION,
					   sizeof(float) * 2);
		memcpy(get_send_msg()->msg->payload, &speed, sizeof(speed));
		memcpy(get_send_msg()->msg->payload + sizeof(speed), &position, sizeof(position));
		get_send_msg()->index = get_send_msg()->msg->header.len;
		rc = 0;
		break;
	case COMMUNICATION_SET_PI:
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT(recv_msg->payload, p_cof);
		COMMUNICATION_CONVERT_BYTE_ARRAY_TO_FLOAT((recv_msg->payload + 4), i_cof);
		get_pi_controller()->kp = p_cof;
		get_pi_controller()->ki = i_cof;
		LOG_INF("set PI %f %f", (double)p_cof, (double)i_cof);
		break;
	default:
		LOG_DBG("invalid command");
		return -EINVAL;
	}

	return rc;
}

static int setup_uart(const struct device *uart, void *user_data)
{
	char data;

	if (!device_is_ready(uart)) {
		return -EIO;
	}

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);
	// flush data
	while (uart_fifo_read(uart, &data, sizeof(data)) > 0) {
		// do nothing
		continue;
	}
	uart_irq_callback_user_data_set(uart, uart_isr_cb, user_data);
	uart_irq_rx_enable(uart);

	return 0;
}

bool *is_speed_control(void)
{
	static bool is_speed = true;
	return &is_speed;
}

static void uart_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	struct communicate_context *thread_ctx = (struct communicate_context *)arg1;
	const struct device *uart = (const struct device *)arg2;
	int rc;

	do {
		k_sem_take(&thread_ctx->msg_in, K_FOREVER);
		LOG_DBG("%s: new msg in", __func__);

		// set recv message to busy state
		atomic_set(&get_recv_msg()->busy, 1);

		rc = message_handler(get_recv_msg()->msg, get_recv_msg()->msg->header.len);
		if (!rc) {
			// printk("response %d bytes\n", get_send_msg()->index);
			atomic_set(&get_send_msg()->done, 1);
			uart_irq_tx_enable(uart);
		} else {
			// release recv message buffer
			// memset(get_recv_msg()->msg, 0, sizeof(struct common_message));
			atomic_set(&get_recv_msg()->busy, 0);
			atomic_set(&get_recv_msg()->done, 0);
			get_recv_msg()->index = 0;
		}

	} while (true);

	return;
}

int communication_init(void)
{
	LOG_INF("init communicate thread");

	k_sem_init(&uart_ctx.msg_in, 0, 1);

	setup_uart(uart1, &uart_ctx);

	k_thread_create(&uart_ctx.communicate_thread, uart_ctx.stack,
			K_KERNEL_STACK_SIZEOF(uart_ctx.stack), uart_thread, &uart_ctx,
			(void *)uart1, NULL, K_LOWEST_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&uart_ctx.communicate_thread, "uart_thread");

	return 0;
}

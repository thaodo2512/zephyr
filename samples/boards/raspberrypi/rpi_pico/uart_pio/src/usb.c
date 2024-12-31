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

	if (atomic_get(&get_recv_msg()->busy) == 1 || atomic_get(&get_recv_msg()->done) == 1) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev) != 1) {
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

	if (get_send_msg()->index < 0 && atomic_get(&get_send_msg()->done) == 1) {
		// complete to send message
		atomic_set(&get_send_msg()->done, 0);
		atomic_set(&get_send_msg()->busy, 0);
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

	// LOG_INF("send 0x%02x - priv = %d - index = %d", *((char *)get_send_msg()->msg +
	// priv_cnt), priv_cnt, get_send_msg()->index);
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

	switch (recv_msg->header.cmd) {
	case COMMUNICATION_GET_SPEED:
		((struct motor_drive_encoder_api *)get_encoder_device()->api)
			->get_speed(get_encoder_device(), &speed);
		COMMUNICATION_SET_RESPONSE(get_send_msg()->msg, COMMUNICATION_GET_SPEED,
					   sizeof(float));
		memcpy(get_send_msg()->msg->payload, &speed, sizeof(float));
		get_send_msg()->index = get_send_msg()->msg->header.len;
		LOG_DBG("get speed");
		rc = 0;
		break;
	case COMMUNICATION_SET_SPEED:
		LOG_DBG("set speed");
		speed = (float)(recv_msg->payload[0] | (recv_msg->payload[1] << 8) |
				(recv_msg->payload[2] << 16) | (recv_msg->payload[3] << 24));
		*get_set_point() = speed;
		rc = 0;
		break;
	case COMMNUICATION_GET_POSITION:
		LOG_DBG("get position");
		break;
	case COMMUNICATION_SET_POSITION:
		LOG_DBG("set position");
		break;
	case COMMUNICATION_SET_P:
		LOG_DBG("set P");
		break;
	case COMMUNICATION_SET_I:
		LOG_DBG("set I");
		break;
	case COMMUNICATION_SET_D:
		LOG_DBG("set D");
		break;
	default:
		LOG_ERR("invalid command");
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
		LOG_HEXDUMP_DBG(get_recv_msg()->msg, get_recv_msg()->index, "uart recv msg");

		rc = message_handler(get_recv_msg()->msg, get_recv_msg()->msg->header.len);
		if (!rc) {
			atomic_set(&get_send_msg()->done, 1);
			uart_irq_tx_enable(uart);
		}

		// LOG_INF("===== start send message len = %d =====",
		// get_send_msg()->msg->header.len); LOG_HEXDUMP_INF(get_send_msg()->msg,
		// get_send_msg()->msg->header.len, "dump test");

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

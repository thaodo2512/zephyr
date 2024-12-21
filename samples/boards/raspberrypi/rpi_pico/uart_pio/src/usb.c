/*
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>
#include <string.h>

#include "common.h"

LOG_MODULE_REGISTER(communicate, LOG_LEVEL_DBG);

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

static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
static struct communicate_context uart_ctx;
static struct common_message recevice_msg = {0};
static struct uart_message uart_recv_msg = {
	.msg = &recevice_msg,
	.index = 0,
	.done = ATOMIC_INIT(0),
	.busy = ATOMIC_INIT(0),
	.error = ATOMIC_INIT(0),
};
static struct uart_message uart_send_msg = {
	.msg = &recevice_msg,
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

/**
 * @brief UART interrupt callback
 *
 * @param uart_dev  uart device
 * @param user_data users's data
 */
static void uart_isr_cb(const struct device *uart_dev, void *user_data)
{
	uint8_t data;
	int rc;
	struct communicate_context *uart_ctx = (struct communicate_context *)user_data;

	if (!uart_dev || !user_data) {
		LOG_ERR("invalid param for %s", __func__);
		return;
	}

	if (uart_irq_update(uart_dev) != 1) {
		return;
	}

	LOG_DBG("uart interrupt");

	if (atomic_get(&get_recv_msg()->busy) == 1 || atomic_get(&get_recv_msg()->done) == 1) {
		// we are busy to handle the old message
		// reject the new message
		LOG_WRN("MCU-uart is busy, reject the new message");
	} else {
		do {
			if (uart_irq_rx_ready(uart_dev) != 1) {
				break;
			}

			rc = uart_fifo_read(uart_dev, &data, sizeof(data));
			if (rc < sizeof(data)) {
				// no more data
				break;
			}

			get_recv_msg()->msg->payload[get_recv_msg()->index] = data;
			get_recv_msg()->index++;
			if (get_recv_msg()->index >= 32) {
				LOG_ERR("MCU-uart message is too long");
				atomic_set(&get_recv_msg()->error, 1);
				break;
			}
		} while (true);
	}

	// check if message is done
	if (get_recv_msg()->msg->header.len == get_recv_msg()->index) {
		atomic_set(&get_recv_msg()->done, 1);
		k_sem_give(&uart_ctx->msg_in);
	}

	// to avoid loop forever in isr, just send 1 char at a time
	do {
		if (atomic_get(&get_send_msg()->error) == 1) {
			break;
		}

		if (atomic_get(&get_send_msg()->done) != 1) {
			// nothing to be done
			break;
		}

		if (get_send_msg()->index < 0) {
			// complete to send message
			atomic_set(&get_send_msg()->done, 1);
			atomic_set(&get_send_msg()->busy, 0);
			break;
		}

		rc = uart_irq_tx_ready(uart_dev);
		if (rc <= 0) {
			break;
		}

		// set buffer to busy state
		atomic_set(&get_send_msg()->busy, 1);
		rc = uart_fifo_fill(uart_dev, get_send_msg()->msg->payload, 1);
		if (rc <= 0) {
			LOG_WRN("could not send message (uart)");
			break;
		}
		get_send_msg()->index--;

	} while (false);

	return;
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

static void uart_thread(void *arg1, void *arg2, void *unused2)
{
	ARG_UNUSED(unused2);

	struct communicate_context *thread_ctx = (struct communicate_context *)arg1;
	const struct device *uart = (const struct device *)arg2;

	while (true) {
		k_sem_take(&thread_ctx->msg_in, K_FOREVER);
		// set recv message to busy state
		atomic_set(&get_recv_msg()->busy, 1);

		// do something here
		LOG_INF("new msg in");
		LOG_HEXDUMP_INF(get_recv_msg()->msg, 32, "uart recv msg");

		uart_irq_tx_enable(uart);

		// release recv message buffer
		memset(get_recv_msg()->msg, 0, 32);
		atomic_set(&get_recv_msg()->busy, 0);
		atomic_set(&get_recv_msg()->done, 0);
	}

	return;
}

int communication_init(void)
{
	LOG_INF("init communicate thread");

	k_sem_init(&uart_ctx.msg_in, 0, 1);

	setup_uart(uart1, &uart_ctx);

	k_thread_create(&uart_ctx.communicate_thread, uart_ctx.stack,
			K_KERNEL_STACK_SIZEOF(uart_ctx.stack), uart_thread, &uart_ctx,
			(void *)uart1, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&uart_ctx.communicate_thread, "uart_thread");
	return 0;
}

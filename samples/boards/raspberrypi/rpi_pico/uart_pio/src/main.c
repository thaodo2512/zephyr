/*
 * Copyright (c) 2023 Yonatan Schachter
 * Copyright (c) 2024 Thao Do
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "common.h"


const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(pio1_uart0));
// const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(pio1_uart1));
static struct common_message msg = { 0 };
static uint32_t recv_counter = 0;
static uint32_t send_counter = 0;
static bool is_recv_msg_done = false;
static uint32_t msg_len = 0;
static bool is_res_msg_ready = false;
static bool is_res_msg_done = false;
static char msg_res[] = "Hello, World!";


static void uart_isr_cb(const struct device *uart_dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t data;
	int rc;
	uint8_t *p_msg = (uint8_t *)&msg;

	if (is_recv_msg_done == true) {
		// start a need message
		recv_counter = 0;
		msg_len = 0;
		is_recv_msg_done = false;
	}

	while (uart_irq_update(uart0) && uart_irq_rx_ready(uart0)) {
		rc = uart_fifo_read(uart0, &data, sizeof(data));
		if (rc < sizeof(data)) {
			// no more data
			break;
		}

		if (recv_counter == offsetof(struct common_header, len)) {
			msg_len = data;
		}

		p_msg[recv_counter] = data;
		recv_counter++;
	}

	if (uart_irq_tx_ready(uart0) && is_res_msg_ready && send_counter < sizeof(msg_res)) {
		send_counter = uart_fifo_fill(uart0, msg_res, sizeof(msg_res));
	}

	if (msg_len && recv_counter >= msg_len) {
		is_recv_msg_done = true;
	}

	if (send_counter >= sizeof(msg_res)) {
		send_counter = 0;
		is_res_msg_done = true;
		is_res_msg_ready = false;
	}

	return;
}

int setup_uart(void)
{
	char data;

	if (!device_is_ready(uart0)) {
		return -EIO;
	}

	uart_irq_rx_disable(uart0);
	uart_irq_tx_disable(uart0);
	// flush data
	while (uart_fifo_read(uart0, &data, sizeof(data)) > 0) {
		// do nothing
		continue;
	}
	uart_irq_callback_set(uart0, uart_isr_cb);
	uart_irq_rx_enable(uart0);

	return 0;
}

int main(void)
{
	int rc;


	rc = setup_uart();
	if (rc) return rc;

	while (1) {
		if (!is_recv_msg_done) {
			continue;
		}

		is_res_msg_ready = true;
	}

	return 0;
}

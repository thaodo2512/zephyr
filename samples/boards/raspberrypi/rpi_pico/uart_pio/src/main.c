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
#include <sample_usbd.h>

#include <zephyr/input/input.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

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

int main(void)
{
	LOG_INF("Booting to raspberry pi pico - bai tap lon 2024");
    LOG_INF("Build time: %s", BUILD_TIME);
    LOG_INF("Commit hash: %s", GIT_COMMIT_HASH);

	int rc;

	rc = communication_init();
	if (rc) {
		LOG_ERR("failed to setup communication - rc = %d", rc);
		return rc;
	}
	// int rc;
	// struct usbd_context *usb_hid_ctx;
	// const struct device *usb_hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);

	// rc = setup_uart();
	// if (rc) {
	// 	LOG_ERR("failed to setup uart %s - rc = %d", uart1->name, rc);
	// 	return rc;
	// }

	// if (!device_is_ready(usb_hid_dev)) {
	// 	LOG_ERR("USB HID device is not ready");
	// 	return -EIO;
	// }

	// while (1) {
	// 	// if (!is_recv_msg_done) {
	// 	// 	continue;
	// 	// }

	// 	// is_res_msg_ready = true;
	// 	// testing
	// 	LOG_INF("send message to uart %s", msg_res);
	// 	uart_irq_tx_enable(uart1);
	// 	k_sleep(K_MSEC(1000));
	// 	// end test
	// }

	while (1) {
		k_sleep(K_MSEC(1000));
	}

	return 0;
}

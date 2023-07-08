/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>

#include "drivers/nrf_sw_servo.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

void main(void)
{
    const struct device *servo = DEVICE_DT_GET(DT_NODELABEL(servo0));
    if (servo == NULL) {
        printk("Failed to get servo dev binding\n");
        return;
    }

    if (!device_is_ready(servo)) {
    	printk("servo device is not ready\n");
        return;
    }

    int ret = servo_write(servo, 100);
    if (!ret) {
        printk("servo_write failed: %d\n", ret);
        return;
    }
}

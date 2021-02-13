/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <string.h>
#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/__assert.h>
#include <drivers/servo.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

K_TIMER_DEFINE(sync_timer, NULL, NULL);

void main(void)
{
    int ret;
    const struct device *dev;

    if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT)) {
        /* Give RTT log time to be flushed before executing tests */
        k_sleep(K_MSEC(500));
    }

    size_t len = z_device_get_all_static(&dev);
    const struct device *dev_end = dev + len;
    while (dev < dev_end) {
        if (z_device_ready(dev)
            && (dev->name != NULL)
            && (strlen(dev->name) != 0)) {
            LOG_INF("Found device: %s", dev->name);
        }
        dev++;
    }

    dev = device_get_binding("SERVO_0");
    if (dev == NULL) {
        LOG_ERR("Failed to get dev binding");
        return;
    }
    LOG_INF("dev is %p, name is %s", dev, dev->name);

    int step=4;
    k_timer_start(&sync_timer, K_MSEC(0), K_MSEC(500));
    while (1) {
    	int value;

    	step = ((step + 1) % 8);
    	switch(step) {
    	case 1:
    		value = 25;
    		break;
    	case 2:
    		value = 50;
    		break;
    	case 3:
    		value = 75;
    		break;
    	case 4:
    		value = 100;
    		break;
        case 5:
            value = 75;
            break;
        case 6:
            value = 50;
            break;
        case 7:
            value = 25;
            break;
        default:
            value = 0;
            break;
    	}

    	ret = servo_write(dev, value);
    	if (ret) {
    		LOG_INF("servo_write failed: %d", ret);
    	}
    	k_timer_status_sync(&sync_timer);
    }
    LOG_INF("exiting");
}

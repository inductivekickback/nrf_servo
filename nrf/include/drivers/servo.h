/**
 * @file servo.h
 *
 * @brief Public API for the servo driver
 */

/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#ifndef ZEPHYR_INCLUDE_SERVO_H_
#define ZEPHYR_INCLUDE_SERVO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr.h>
#include <device.h>
#include <stdbool.h>

#define SERVO_MIN_VALUE      0
#define SERVO_NEUTRAL_VALUE	 50
#define SERVO_MAX_VALUE      100

typedef int (*servo_init_t) (const struct device *dev);
typedef int (*servo_write_t)(const struct device *dev, uint8_t  value);
typedef int (*servo_read_t) (const struct device *dev, uint8_t *value);

/**
 * @brief Servo driver API
 *
 * This is the API all servo drivers must expose.
 */
struct servo_driver_api {
	servo_init_t  init;
	servo_write_t write;
	servo_read_t  read;
};

static inline int servo_init(const struct device *dev)
{
	struct servo_driver_api *api;

	if (dev == NULL) {
		return -EINVAL;
	}

	api = (struct servo_driver_api*)dev->api;

	if (api->init == NULL) {
		return -ENOTSUP;
	}
	return api->init(dev);
}

static inline int servo_write(const struct device *dev, uint8_t value)
{
	struct servo_driver_api *api;

	if (dev == NULL) {
		return -EINVAL;
	}

	api = (struct servo_driver_api*)dev->api;

	if (api->write == NULL) {
		return -ENOTSUP;
	}
	return api->write(dev, value);
}

static inline int servo_read(const struct device *dev, uint8_t *value)
{
	struct servo_driver_api *api;

	if (dev == NULL) {
		return -EINVAL;
	}

	api = (struct servo_driver_api*)dev->api;

	if (api->read == NULL) {
		return -ENOTSUP;
	}
	return api->read(dev, value);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SERVO_H_ */

/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define DT_DRV_COMPAT nrf_servo

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <drivers/nrf_servo.h>

#include <hal/nrf_gpio.h>
#include <nrfx_pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nrf_servo, CONFIG_NRF_SERVO_LOG_LEVEL);

/* If this flag is not set in the sequence values that are passed to
   nrf_drv_pwm then the polarity of the PWM waveform will be inverted. */
#define POLARITY_BIT  (0x8000UL)

// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/100 + (min))
#define PAM(value, min, max) (((value) - (min)) * 100 / ((max) - (min)))

// Structs of this type need to kept in the global portion (static) of RAM
// (not const) because they are accessed by EasyDMA.
typedef struct
{
    nrfx_pwm_t pwm_instance;
    nrf_pwm_values_individual_t pwm_values;
    bool ready;
} servo_group_t;

struct servo_data {
	bool ready;
};

struct servo_cfg {
    uint32_t pin;
    uint8_t  pwm_index;
    uint8_t  pwm_channel;
    uint8_t  init_value;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
};

static servo_group_t m_avail_pwms[] = {
#if CONFIG_NRF_SERVO_ALLOW_PWM0
	{
		.pwm_instance = NRFX_PWM_INSTANCE(0),
        .ready        = false,
    },
#endif
#if CONFIG_NRF_SERVO_ALLOW_PWM1
	{
		.pwm_instance = NRFX_PWM_INSTANCE(1),
        .ready        = false,
    },
#endif
#if CONFIG_NRF_SERVO_ALLOW_PWM2
	{
		.pwm_instance = NRFX_PWM_INSTANCE(2),
        .ready        = false,
    },
#endif
#if CONFIG_NRF_SERVO_ALLOW_PWM3
	{
		.pwm_instance = NRFX_PWM_INSTANCE(3),
        .ready        = false,
    },
#endif
};

#define NUM_AVAIL_PWMS (sizeof(m_avail_pwms)/sizeof(servo_group_t))

static int channel_get(nrf_pwm_values_individual_t *p_values,
	                     const struct servo_cfg *p_cfg,
	                     uint8_t *p_value)
{
	uint16_t *p_channel;

    switch (p_cfg->pwm_channel) {
    case 0:
    	p_channel = &p_values->channel_0;
    	break;
    case 1:
    	p_channel = &p_values->channel_1;
    	break;
    case 2:
    	p_channel = &p_values->channel_2;
    	break;
    case 3:
    	p_channel = &p_values->channel_3;
    	break;
    default:
    	return -EINVAL;
    }
    *p_value = (uint8_t)PAM(*p_channel & ~POLARITY_BIT, p_cfg->min_pulse_us, p_cfg->max_pulse_us);
    return 0;
}

static int channel_set(nrf_pwm_values_individual_t *p_values,
	                     const struct servo_cfg *p_cfg,
	                     uint8_t value)
{
	uint16_t *p_channel;

	if (SERVO_MAX_VALUE < value)
	{
		return -EINVAL;
	}

    switch (p_cfg->pwm_channel) {
    case 0:
    	p_channel = &p_values->channel_0;
    	break;
    case 1:
    	p_channel = &p_values->channel_1;
    	break;
    case 2:
    	p_channel = &p_values->channel_2;
    	break;
    case 3:
    	p_channel = &p_values->channel_3;
    	break;
    default:
    	return -EINVAL;
    }
    *p_channel = (MAP(value, p_cfg->min_pulse_us, p_cfg->max_pulse_us) | POLARITY_BIT);
    return 0;
}

static int nrf_servo_init(const struct device *dev)
{
    int err;
    nrfx_pwm_t *p_inst;

    const struct servo_cfg *p_cfg  = dev->config;
    struct servo_data      *p_data = dev->data;

    if (unlikely(p_data->ready)) {
        /* Already initialized */
        return 0;
    }

    if (NUM_AVAIL_PWMS <= p_cfg->pwm_index) {
    	goto ERR_EXIT;
    }

    p_inst = &m_avail_pwms[p_cfg->pwm_index].pwm_instance;

    if (!m_avail_pwms[p_cfg->pwm_index].ready) {
    	nrfx_err_t nrfx_err;
        nrfx_pwm_config_t const pwm_config = {
            .output_pins = {
                NRFX_PWM_PIN_NOT_USED,
                NRFX_PWM_PIN_NOT_USED,
                NRFX_PWM_PIN_NOT_USED,
                NRFX_PWM_PIN_NOT_USED
            },
            .irq_priority = 0,
            .base_clock   = NRF_PWM_CLK_1MHz,
            .count_mode   = NRF_PWM_MODE_UP,
            .top_value    = 20000, // 20ms
            .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode    = NRF_PWM_STEP_AUTO
        };
    
        nrfx_err = nrfx_pwm_init(p_inst, &pwm_config, NULL, NULL);
        if (NRFX_SUCCESS != nrfx_err)
        {
            goto ERR_EXIT;
        }

      	nrf_pwm_sequence_t const seq = {
	        .values.p_individual = &m_avail_pwms[p_cfg->pwm_index].pwm_values,
	        .length              = 4,
	        .repeats             = 0,
	        .end_delay           = 0
	    };
        err = nrfx_pwm_simple_playback(p_inst, &seq, 1, NRFX_PWM_FLAG_LOOP);

    	/* NOTE: nrfx_pwm_simple_playback returns NULL instead of NRFX_SUCCESS. */
    	if (err) {
    		goto ERR_EXIT;
    	}

    	m_avail_pwms[p_cfg->pwm_index].ready = true;
    }

    err = channel_set(&m_avail_pwms[p_cfg->pwm_index].pwm_values, p_cfg, p_cfg->init_value);
    if (0 != err) {
    	goto ERR_EXIT;
    }

    nrf_gpio_pin_clear(p_cfg->pin);
    nrf_gpio_cfg_output(p_cfg->pin);
    p_inst->p_registers->PSEL.OUT[p_cfg->pwm_channel] = p_cfg->pin;

    p_data->ready = true;
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static int nrf_servo_write(const struct device *dev, uint8_t value)
{
	const struct servo_cfg *p_cfg  = dev->config;
	struct servo_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = channel_set(&m_avail_pwms[p_cfg->pwm_index].pwm_values, p_cfg, value);
    if (0 != err) {
    	return err;
    }

    return 0;
}

static int nrf_servo_read(const struct device *dev, uint8_t *value)
{
	const struct servo_cfg *p_cfg  = dev->config;
	struct servo_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = channel_get(&m_avail_pwms[p_cfg->pwm_index].pwm_values, p_cfg, value);
    if (0 != err) {
    	return err;
    }

    return 0;
}

static const struct servo_driver_api servo_driver_api = {
    .init  = nrf_servo_init,
    .write = nrf_servo_write,
    .read  = nrf_servo_read,
};

#define INST(num) DT_INST(num, nrf_servo)

#define SERVO_DEVICE(n) \
    static const struct servo_cfg servo_cfg_##n = { \
        .pin          = DT_PROP(INST(n), pin), \
        .init_value   = DT_PROP(INST(n), init_value), \
        .min_pulse_us = DT_PROP(INST(n), min_pulse_us), \
        .max_pulse_us = DT_PROP(INST(n), max_pulse_us), \
        .pwm_channel  = (n % NRF_PWM_CHANNEL_COUNT), \
        .pwm_index    = (n / NRF_PWM_CHANNEL_COUNT) \
    }; \
    static struct servo_data servo_data_##n; \
    DEVICE_DT_DEFINE(INST(n), \
            nrf_servo_init, \
            NULL, \
            &servo_data_##n, \
            &servo_cfg_##n, \
            POST_KERNEL, \
            CONFIG_NRF_SERVO_INIT_PRIORITY, \
            &servo_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SERVO_DEVICE)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "nRF-Servo driver enabled without any devices"
#endif

/*
 * Copyright (c) 2023 CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hls_lfcd2

#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>

#include "hls-lfcd2.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(HLS_LFCD2, CONFIG_SENSOR_LOG_LEVEL);

static void process_data(const struct device *dev)
{
    struct hls_lfcd2_data *data = dev->data;

    for (int i = 0; i < HLS_LFCD2_BUF_SIZE; i++) {
        data->hls_lfcd2_buffer[i].intensity = data->buffer[i];
        data->hls_lfcd2_buffer[i].distance = data->buffer[i];
    }

    struct sensor_trigger drdy_trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    if (data->handler != NULL) {
        data->handler(dev, &drdy_trigger);
    }
}

static int hls_lfcd2_init_pwm(const struct device *dev)
{
    const struct hls_lfcd2_config *cfg = dev->config;
    struct hls_lfcd2_data *data = dev->data;

    const struct pwm_dt_spec *led = &cfg->pwm;

    if (!device_is_ready(led->pwm_dev)) {
        LOG_ERR("PWM device %s is not ready", led->pwm_dev->name);
        return -ENODEV;
    }

    // Set duty cycle to 50%
    pwm_set_pulse_dt(&led->pwm_dev, 500000);

    return 0;
}

static void hls_lfcd2_uart_callback(const struct device *dev, void *user_data)
{
    const struct device *uart_dev = user_data;
    struct hls_lfcd2_data *data = uart_dev->data;
    static int len = 0;
    static int sync = 0;

    if ((uart_irq_update(uart_dev) & UART_RX_RDY) != 0) {
        while (uart_irq_rx_ready(dev)) {
            if (sync == 0) {
                if (uart_fifo_read(dev, &data->buffer[len], 1) == 1) {
                    if (data->buffer[len] == HLS_LFCD2_SYNC_BYTE) {
                        sync = 1;
                        len++;
                    }
                }
            } else if (sync == 1) {
                if (uart_fifo_read(dev, &data->buffer[len], 1) == 1) {
                    if (data->buffer[len] == 0xA0) {
                        sync = 2;
                        len++;
                    } else {
                        sync = 0;
                        len = 0;
                    }
                }
            } else if (sync == 2) {
                if (uart_fifo_read(dev, &data->buffer[len], 1) == 1) {
                    if (len >= HLS_LFCD2_BUF_SIZE) {
                        sync = 0;
                        len = 0;
                        process_data(dev);
                    } else {
                        len++;
                    }
                }
            }
        }
    }
}

static int hls_lfcd2_channel_get(
        const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct hls_lfcd2_data *data = dev->data;

    switch (chan) {
        case SENSOR_CHAN_DISTANCE:
            for (int i = 0; i < HLS_LFCD2_BUF_SIZE; i++) {
                val->val1 = data->hls_lfcd2_buffer[i].intensity;
                val->val2 = data->hls_lfcd2_buffer[i].distance;
                val++;
            }
            break;
        case SENSOR_CHAN_RPM:
            val->val1 = data->rpm;
            val->val2 = 0;
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static int hls_lfcd2_init(const struct device *dev)
{
    const struct hls_lfcd2_config *cfg = dev->config;
    int ret;

    if (!device_is_ready(cfg->dev)) {
        LOG_ERR("UART device %s is not ready", cfg->dev->name);
        return -ENODEV;
    }

    uart_irq_callback_user_data_set(cfg->dev, hls_lfcd2_uart_callback, (void *)dev);

    ret = hls_lfcd2_init_pwm(dev);

    if (ret < 0) {
        LOG_ERR("Failed to initialize PWM");
        return ret;
    }

    return 0;
}

static const struct sensor_driver_api hls_lfcd2_api = {
    // .sample_fetch = hls_lfcd2_sample_fetch,
    .channel_get = hls_lfcd2_channel_get,
};

#define HLS_LFCD2_INIT(n)                                                                          \
    static struct hls_lfcd2_data hls_lfcd2_data_##n;                                               \
                                                                                                   \
    static const struct hls_lfcd2_config hls_lfcd2_config_##n                                      \
            = { .dev = DEVICE_DT_GET(DT_INST_BUS(n)), .pwm = GPIO_DT_SPEC_INST_GET_OR(n, pwm, {}),
}
;

DEVICE_DT_INST_DEFINE(n,
        hls_lfcd2_init,
        NULL,
        &hls_lfcd2_data_##n,
        &hls_lfcd2_config_##n,
        POST_KERNEL,
        CONFIG_SENSOR_INIT_PRIORITY,
        &hls_lfcd2_api);

DT_INST_FOREACH_STATUS_OKAY(HLS_LFCD2_INIT)
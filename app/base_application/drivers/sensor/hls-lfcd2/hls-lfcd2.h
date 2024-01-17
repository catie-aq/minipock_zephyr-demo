/*
 * Copyright (c) 2023 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_HLS_LFCD2_H_
#define ZEPHYR_DRIVERS_SENSOR_HLS_LFCD2_H_

#define HLS_LFCD2_BUF_SIZE 2520
#define HLS_LFCD2_SYNC_BYTE 0xFA

struct hls_lfcd2_buf {
    uint16_t intensity;
    uint16_t distance;
};

struct hls_lfcd2_config {
    const struct device *dev;
    struct pwm_dt_spec pwm;
};

struct hls_lfcd2_data {
    const struct device *gpio_dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

    uint8_t buffer[HLS_LFCD2_BUF_SIZE];
    struct hls_lfcd2_buf hls_lfcd2_buffer[HLS_LFCD2_BUF_SIZE];
    uint16_t idx;

    uint16_t rpm;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_HLS_LFCD2_H_ */

/*
 * Copyright (c) 2023 CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_SENSOR_LOG_LEVEL);

int main(void)
{
    const struct device *sensor;

    sensor = DEVICE_DT_GET(DT_NODELABEL(lidar0));
    if (!device_is_ready(sensor)) {
        LOG_ERR("Sensor not ready");
        return 0;
    }

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}

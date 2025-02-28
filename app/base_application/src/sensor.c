#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "paa5160e1_global.h"

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);

static struct base_interface_trigger *sensor_interface_trigger;

static void trigger_handler_sensor(const struct device *dev, struct sensor_trigger *trigger)
{
    struct sensor_value value;
    int ret;

    ret = sensor_sample_fetch(dev);
    if (ret) {
        printk("Failed to fetch sensor sample (%d)\n", ret);
        return;
    }

    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_X, &value);
    printk("X: %d.%06d mm", value.val1, value.val2);
    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_Y, &value);
    printk("Y: %d.%06d mm", value.val1, value.val2);
    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_H, &value);
    printk("H: %d deg (x10^1) \n", value.val1);
}

void sensor_thread(void)
{
    printk("Sensor thread started");

    while (1) {
        printk("Sensor thread running\n");
        k_sleep(K_MSEC(100000));
    }
}

int init_sensor(struct base_interface_trigger *trigger)
{
    LOG_DBG("Initializing sensor");
    sensor_interface_trigger = trigger;

    const struct device *sensor_dev;
    struct sensor_trigger trig;
    int ret;

    sensor_dev = DEVICE_DT_GET(DT_NODELABEL(paa5160e1));
    if (!sensor_dev) {
        LOG_ERR("No device found\n");
        return;
    }

    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ALL;

    ret = sensor_trigger_set(sensor_dev, &trig, trigger_handler_sensor);
    if (ret) {
        LOG_ERR("Failed to set sensor trigger (%d)\n", ret);
        return;
    }

    return 0;
}

K_THREAD_DEFINE(sensor_thread_id, 1024, sensor_thread, NULL, NULL, NULL, 7, 0, 0);

// 		sensor_sample_fetch(sensor);
//	    struct sensor_value value;
// 		sensor_channel_get(sensor, PAA5160E1_SENSOR_CHAN_X, &value);

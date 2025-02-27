#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);

static void sensor_callback(const struct device *dev, struct sensor_trigger *trigger)
{
    struct sensor_value value;
    int ret;

    ret = sensor_sample_fetch(dev);
    if (ret) {
        printk("Failed to fetch sensor sample (%d)\n", ret);
        return;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_ALL, &value);
    if (ret) {
        printk("Failed to get sensor channel value (%d)\n", ret);
        return;
    }

    printk("Sensor value: %d.%06d\n", value.val1, value.val2);
}

void sensor_thread(void)
{
    printk("Sensor thread started");

    const struct device *sensor_dev;
    struct sensor_trigger trig;
    int ret;

    sensor_dev = DEVICE_DT_GET(DT_NODELABEL(paa5160e1));
    if (!sensor_dev) {
        printk("No device found\n");
        return;
    }

    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ALL;

    ret = sensor_trigger_set(sensor_dev, &trig, sensor_callback);
    if (ret) {
        printk("Failed to set sensor trigger (%d)\n", ret);
        return;
    }

    while (1) {
        printk("Sensor thread running\n");
        k_sleep(K_MSEC(1));
    }
}

K_THREAD_DEFINE(sensor_thread_id, 1024, sensor_thread, NULL, NULL, NULL, 7, 0, 0);

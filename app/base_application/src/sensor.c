#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "base_interface.h" // Include the header file that defines struct base_interface_trigger
#include "paa5160e1_global.h"

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);

static struct base_interface_trigger *sensor_interface_trigger;

#define NB_MSG 11

struct optic_sensor_data {
    struct sensor_value x;
    struct sensor_value y;
    struct sensor_value h;
};

static struct optic_sensor_data optic_sensor_msg[NB_MSG];
K_MSGQ_DEFINE(optic_sensor_msgq, sizeof(struct optic_sensor_data), 20, 4);

static void trigger_handler_sensor(const struct device *dev, struct sensor_trigger *trigger)
{
    int ret;
    struct sensor_value value;
    struct optic_sensor_data optic_sensor_data;

    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_X, &value);
    // printk("X: %d.%06d mm", value.val1, value.val2);
    optic_sensor_data.x = value;

    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_Y, &value);
    // printk("Y: %d.%06d mm", value.val1, value.val2);
    optic_sensor_data.y = value;

    sensor_channel_get(dev, PAA5160E1_SENSOR_CHAN_H, &value);
    printk("H: %d \n", value.val1);
    optic_sensor_data.h = value;

    while (k_msgq_put(&optic_sensor_msgq, &optic_sensor_data, K_NO_WAIT) != 0) 
    {
        k_msgq_purge(&optic_sensor_msgq);
    }
}

void optic_sensor_thread(void)
{
    LOG_DBG("Sensor thread started");

    while (true) {
        struct optic_sensor_data optic_sensor_data;
        k_msgq_get(&optic_sensor_msgq, &optic_sensor_data, K_FOREVER);

        if (sensor_interface_trigger->odometry_callback != NULL) {
            sensor_interface_trigger->odometry_callback(((float)optic_sensor_data.x.val1)/1000.0,
                    ((float)optic_sensor_data.y.val1)/1000.0,
                    ((float)optic_sensor_data.h.val1 * 3.14159265359 / 1800.0 ));
        }
    }
}

int init_optic_sensor(struct base_interface_trigger *trigger)
{
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

K_THREAD_DEFINE(sensor_thread_id, 2048, optic_sensor_thread, NULL, NULL, NULL, 7, 0, 0);

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rcl/error_handling.h>

#include "scan.h"

LOG_MODULE_REGISTER(scan, LOG_LEVEL_INF);

static struct scan_trigger *scan_trigger;

static struct lidar_data lidar_msg[NB_MSG];
int lidar_msg_index = 0;

K_MSGQ_DEFINE(lidar_msgq, sizeof(struct lidar_data), 20, 4);

static const struct device *lidar = DEVICE_DT_GET(DT_NODELABEL(lidar0));

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
    struct sensor_value val[15];

    struct lidar_data lidar_data;

    sensor_channel_get(lidar, SENSOR_CHAN_DISTANCE, val);

    lidar_data.start_angle = val[1].val1 / 100 * 3.14 / 180;
    lidar_data.end_angle = val[1].val2 / 100 * 3.14 / 180;

    for (int i = 0; i < NB_POINTS_PER_MSG; i++) {
        lidar_data.ranges[i] = (double)val[i + 2].val1 / 1000.;
        lidar_data.intensities[i] = (double)val[i + 2].val2;
    }

    while (k_msgq_put(&lidar_msgq, &lidar_data, K_NO_WAIT) != 0) {
        k_msgq_purge(&lidar_msgq);
    }
}

void send_lidar_data(void *, void *, void *)
{
    static float range_to_send[NB_POINTS];
    static float intensity_to_send[NB_POINTS];

    struct lidar_data lidar_data;

    while (true) {
        k_msgq_get(&lidar_msgq, &lidar_data, K_FOREVER);

        lidar_msg[lidar_msg_index] = lidar_data;
        lidar_msg_index++;

        if (lidar_msg_index >= NB_MSG) {
            lidar_msg_index = 0;

            for (int i = 0; i < NB_MSG; i++) {
                for (int j = 0; j < NB_POINTS_PER_MSG; j++) {
                    range_to_send[i * NB_POINTS_PER_MSG + j] = lidar_msg[i].ranges[j];
                    intensity_to_send[i * NB_POINTS_PER_MSG + j] = lidar_msg[i].intensities[j];
                }
            }

            if (scan_trigger->lidar_scan_callback != NULL) {
                scan_trigger->lidar_scan_callback(range_to_send,
                        intensity_to_send,
                        lidar_msg[0].start_angle,
                        lidar_msg[NB_MSG - 1].end_angle);
            }
        }
    }
}

int init_scan(struct scan_trigger *trigger)
{
    int ret = 0;

    scan_trigger = trigger;

    // Initialize LiDAR
    if (!device_is_ready(lidar)) {
        LOG_ERR("LiDAR device not ready\n");
        return -ENODEV;
    }

    static struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_DISTANCE,
    };

    ret = sensor_trigger_set(lidar, &trig, trigger_handler);
    if (ret) {
        LOG_ERR("Failed to set trigger\n");
        return ret;
    }

    return 0;
}

K_THREAD_DEFINE(scan_thread, 8162, send_lidar_data, NULL, NULL, NULL, 10, 0, 0);

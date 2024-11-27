#include "common.h"
#include <sensor_msgs/msg/laser_scan.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rcl/error_handling.h>

#include "scan.h"

LOG_MODULE_REGISTER(scan, LOG_LEVEL_INF);

#define NB_MSG 11
#define NB_POINTS_PER_MSG 12
#define NB_POINTS (NB_MSG * NB_POINTS_PER_MSG)

struct lidar_data {
    double ranges[12];
    double intensities[12];
    double start_angle;
    double end_angle;
};
static struct lidar_data lidar_msg[NB_MSG];
int lidar_msg_index = 0;

K_MSGQ_DEFINE(lidar_msgq, sizeof(struct lidar_data), 20, 4);

static rcl_publisher_t scan_publisher;

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

    static sensor_msgs__msg__LaserScan scan;

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

            char frame_id[50];
            snprintf(frame_id, sizeof(frame_id), "%s/lds_01_link", CONFIG_ROS_NAMESPACE);
            scan.header.frame_id.data = frame_id;
            scan.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
            scan.header.stamp.nanosec
                    = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

            double angle_increment = 0;
            if (lidar_msg[0].start_angle > lidar_msg[NB_MSG - 1].end_angle) {
                angle_increment
                        = (6.28 - lidar_msg[0].start_angle + lidar_msg[NB_MSG - 1].end_angle)
                        / (double)NB_POINTS;
            } else {
                angle_increment = (lidar_msg[NB_MSG - 1].end_angle - lidar_msg[0].start_angle)
                        / (double)NB_POINTS;
            }

            scan.angle_increment = angle_increment;
            scan.time_increment = 0;
            scan.scan_time = 0.1;
            scan.range_min = 0.02;
            scan.range_max = 12;

            scan.ranges.data = range_to_send;
            scan.intensities.data = intensity_to_send;
            scan.ranges.size = NB_POINTS;
            scan.intensities.size = NB_POINTS;

            scan.angle_min = lidar_msg[0].start_angle;
            scan.angle_max = lidar_msg[NB_MSG - 1].end_angle;

            if (agent_connected) {
                int ret = rcl_publish(&scan_publisher, &scan, NULL);
                if (ret != RCL_RET_OK) {
                    LOG_ERR("Failed to publish message: %d\n", ret);
                }
            }
        }
    }
}

int init_scan(rcl_node_t *node)
{
    int ret = 0;

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

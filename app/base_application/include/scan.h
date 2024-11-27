#ifndef SCAN_H
#define SCAN_H

#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>

#define NB_MSG 11
#define NB_POINTS_PER_MSG 12
#define NB_POINTS (NB_MSG * NB_POINTS_PER_MSG)

struct lidar_data {
    double ranges[12];
    double intensities[12];
    double start_angle;
    double end_angle;
};

struct scan_callbacks {
    void (*lidar_scan_callback)(const float *range_to_send,
            const float *intensity_to_send,
            const double start_angle,
            const double end_angle);
};

int init_scan(struct scan_callbacks *callbacks);

#endif // SCAN_H

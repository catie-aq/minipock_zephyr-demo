#ifndef SCAN_H
#define SCAN_H

#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>

int init_scan(rcl_node_t *node);

#endif // SCAN_H

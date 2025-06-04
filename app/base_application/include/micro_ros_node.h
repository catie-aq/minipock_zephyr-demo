#ifndef MICRO_ROS_NODE_H_
#define MICRO_ROS_NODE_H_

enum states { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };

int micro_ros_node_get_last_version(void);

void disable_cmd_vel(void);

void enable_cmd_vel(void);

const char *get_micro_ros_node_status_string(enum states state);

enum states get_micro_ros_node_status(void);

int init_micro_ros_transport(void);

#endif // MICRO_ROS_NODE_H_

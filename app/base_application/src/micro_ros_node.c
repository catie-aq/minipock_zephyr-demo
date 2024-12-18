#include <zephyr/kernel.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/init_options.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <zephyr/logging/log.h>

#include "base_interface.h"
#include "micro_ros_node.h"
#include "scan.h"

LOG_MODULE_REGISTER(micro_ros_node, LOG_LEVEL_INF);

static rclc_executor_t executor;
static rcl_publisher_t odom_publisher;
static rcl_publisher_t scan_publisher;
static rcl_subscription_t cmd_vel_subscriber;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static struct base_interface_trigger base_callback;
static struct scan_trigger scan_callback;

static geometry_msgs__msg__Twist cmd_vel_twist_msg;
static uint64_t ros_timestamp;

void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *uros_msg = (const geometry_msgs__msg__Twist *)msgin;

    send_command(uros_msg->linear.x,
            uros_msg->linear.y,
            uros_msg->linear.z,
            uros_msg->angular.x,
            uros_msg->angular.y,
            uros_msg->angular.z);
}

void lidar_scan_callback(const float *range_to_send,
        const float *intensity_to_send,
        const double start_angle,
        const double end_angle)
{
    static sensor_msgs__msg__LaserScan scan;

    char frame_id[50];
    snprintf(frame_id, sizeof(frame_id), "%s/lds_01_link", CONFIG_ROS_NAMESPACE);
    scan.header.frame_id.data = frame_id;
    scan.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
    scan.header.stamp.nanosec = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

    double angle_increment = 0;
    if (start_angle > end_angle) {
        angle_increment = (6.28 - start_angle + end_angle) / (double)NB_POINTS;
    } else {
        angle_increment = (end_angle - start_angle) / (double)NB_POINTS;
    }

    scan.angle_increment = angle_increment;
    scan.time_increment = 0;
    scan.scan_time = 0.1;
    scan.range_min = 0.02;
    scan.range_max = 12;

    memcpy(scan.ranges.data, range_to_send, NB_POINTS * sizeof(float));
    memcpy(scan.intensities.data, intensity_to_send, NB_POINTS * sizeof(float));
    scan.ranges.size = NB_POINTS;
    scan.intensities.size = NB_POINTS;

    scan.angle_min = start_angle;
    scan.angle_max = end_angle;

    if (rcl_publish(&scan_publisher, &scan, NULL) != RCL_RET_OK) {
        LOG_ERR("Failed to publish message");
    }
}

void send_odometry_callback(float x, float y, float theta)
{
    // Convert x, y, theta to quaternion
    float cy = cos((double)theta * 0.5);
    float sy = sin((double)theta * 0.5);
    float cp = cos(0.0 * 0.5);
    float sp = sin(0.0 * 0.5);
    float cr = cos(0.0 * 0.5);
    float sr = sin(0.0 * 0.5);

    float q0 = cy * cp * cr + sy * sp * sr;
    float q1 = cy * cp * sr - sy * sp * cr;
    float q2 = sy * cp * sr + cy * sp * cr;
    float q3 = sy * cp * cr - cy * sp * sr;

    // Convert to ROS message
    static geometry_msgs__msg__PoseStamped pose_stamped_msg;

    char frame_id[50];
    snprintf(frame_id, sizeof(frame_id), "%s/odom", CONFIG_ROS_NAMESPACE);
    pose_stamped_msg.header.frame_id.data = frame_id;

    pose_stamped_msg.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
    pose_stamped_msg.header.stamp.nanosec
            = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

    pose_stamped_msg.pose.position.x = (float)x;
    pose_stamped_msg.pose.position.y = (float)y;
    pose_stamped_msg.pose.position.z = 0;

    pose_stamped_msg.pose.orientation.w = q0;
    pose_stamped_msg.pose.orientation.x = q1;
    pose_stamped_msg.pose.orientation.y = q2;
    pose_stamped_msg.pose.orientation.z = q3;

    if (rcl_publish(&odom_publisher, &pose_stamped_msg, NULL) != RCL_RET_OK) {
        LOG_ERR("Failed to publish odometry message");
    }
}

void init_odometry_publisher(rcl_node_t *node)
{
    char odom_topic_name[50];

    snprintf(odom_topic_name, sizeof(odom_topic_name), "/%s/odom_raw", CONFIG_ROS_NAMESPACE);
    rclc_publisher_init_best_effort(&odom_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            odom_topic_name);
}

void init_scan_publisher(rcl_node_t *node)
{
    char scan_topic_name[50];
    snprintf(scan_topic_name, sizeof(scan_topic_name), "/%s/scan_raw", CONFIG_ROS_NAMESPACE);
    static rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    rclc_publisher_init(&scan_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            scan_topic_name,
            &custom_qos_profile);
}

void init_cmd_vel_subscriber(rcl_node_t *node)
{
    char cmd_vel_topic_name[50];
    snprintf(cmd_vel_topic_name, sizeof(cmd_vel_topic_name), "/%s/cmd_vel", CONFIG_ROS_NAMESPACE);
    rclc_subscription_init_best_effort(&cmd_vel_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cmd_vel_topic_name);
}

int init_micro_ros_node(void)
{
    allocator = rcl_get_default_allocator();

    // Initialize micro-ROS with options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        LOG_ERR("Failed to initialize init options");
        return -1;
    }

    if (rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID) != RCL_RET_OK) {
        LOG_ERR("Failed to set domain id");
        return -1;
    }

    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)
            != RCL_RET_OK) {
        LOG_ERR("Failed to initialize support");
        return -1;
    }

    // Create node
    if (rclc_node_init_default(&node, CONFIG_ROS_NAMESPACE, "", &support) != RCL_RET_OK) {
        LOG_ERR("Failed to create node");
        return -1;
    }

    // Initialize publishers
    init_odometry_publisher(&node);
    init_scan_publisher(&node);
    init_cmd_vel_subscriber(&node);

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator)) {
        LOG_ERR("Failed to initialize executor");
        return -1;
    }
    if (rclc_executor_add_subscription(&executor,
                &cmd_vel_subscriber,
                &cmd_vel_twist_msg,
                &subscription_callback,
                ON_NEW_DATA)) {
        LOG_ERR("Failed to add subscription to executor");
        return -1;
    }

    // Synchronize time
    rmw_uros_sync_session(1000);
    ros_timestamp = rmw_uros_epoch_millis() - k_uptime_get();

    // Initialize base interface
    base_callback.odometry_callback = send_odometry_callback;

    init_base_interface(&base_callback);

    // Initialize scan
    scan_callback.lidar_scan_callback = lidar_scan_callback;

    init_scan(&scan_callback);

    return 0;
}

void spin_micro_ros_node(void)
{
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        k_usleep(10000);
    }
}

K_THREAD_DEFINE(micro_ros_spin_thread, 8192, spin_micro_ros_node, NULL, NULL, NULL, 3, 0, 0);

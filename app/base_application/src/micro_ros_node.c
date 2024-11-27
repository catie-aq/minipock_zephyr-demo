#include <zephyr/kernel.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "base_interface.h"
#include "micro_ros_node.h"

rclc_executor_t executor;
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_twist_msg;
uint64_t ros_timestamp;

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

void send_odometry_callback(float x, float y, float theta)
{
    // Convert x, y, theta to quaternion
    float cy = cos((float)theta * 0.5);
    float sy = sin((float)theta * 0.5);
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
        printf("Failed to publish odometry message\n");
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
    rcl_publisher_t scan_publisher;
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

void init_micro_ros_node(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize micro-ROS with options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Create node
    rcl_node_t node;
    rclc_node_init_default(&node, CONFIG_ROS_NAMESPACE, "", &support);

    // Initialize publishers
    init_odometry_publisher(&node);
    init_scan_publisher(&node);
    init_cmd_vel_subscriber(&node);

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor,
            &cmd_vel_subscriber,
            &cmd_vel_twist_msg,
            &subscription_callback,
            ON_NEW_DATA);

    // Synchronize time
    rmw_uros_sync_session(1000);
    ros_timestamp = rmw_uros_epoch_millis() - k_uptime_get();

    // Initialize motor interface
    struct base_interface_callbacks base_callback;
    base_callback.send_odometry_callback = send_odometry_callback;

    init_base_interface(&base_callback);
}

void spin_micro_ros_node(void)
{
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }
}

K_THREAD_DEFINE(micro_ros_spin_thread, 1024, spin_micro_ros_node, NULL, NULL, NULL, 7, 0, 0);

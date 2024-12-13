#include <zephyr/kernel.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <microros_transports.h>
#include <rcl/init_options.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <zephyr/logging/log.h>

#include "base_interface.h"
#include "micro_ros_node.h"
#include "scan.h"

LOG_MODULE_REGISTER(micro_ros_node, LOG_LEVEL_DBG);

static rclc_executor_t executor;
static rcl_publisher_t odom_publisher;
static rcl_publisher_t scan_publisher;
static rcl_subscription_t cmd_vel_subscriber;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_init_options_t init_options;
static rcl_client_t client;

static struct base_interface_trigger base_callback;
static struct scan_trigger scan_callback;

static geometry_msgs__msg__Twist cmd_vel_twist_msg;
static uint64_t ros_timestamp;

bool iniatialized = true;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

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

    if (state == AGENT_CONNECTED) {
        if (rcl_publish(&scan_publisher, &scan, NULL) != RCL_RET_OK) {
            LOG_ERR("Failed to publish message");
        }
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

    if (state == AGENT_CONNECTED) {
        if (rcl_publish(&odom_publisher, &pose_stamped_msg, NULL) != RCL_RET_OK) {
            LOG_ERR("Failed to publish odometry message");
        }
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

void destroy_micro_ros_node(void)
{
    LOG_INF("Destroying micro-ROS node");

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    if (rcl_publisher_fini(&odom_publisher, &node) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy odometry publisher");
    }

    if (rcl_publisher_fini(&scan_publisher, &node) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy scan publisher");
    }

    if (rcl_subscription_fini(&cmd_vel_subscriber, &node) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy cmd_vel subscriber");
    }

    if (rclc_executor_fini(&executor) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy executor");
    }

    if (rcl_node_fini(&node) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy node");
    }

    if (rclc_support_fini(&support) != RCL_RET_OK) {
        LOG_ERR("Failed to destroy support");
    }
}

int init_micro_ros_transport(void)
{
    static zephyr_transport_params_t agent_param = { { 0, 0, 0 }, "192.168.1.3", "8888" };

    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)&agent_param,
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);
}

int init_micro_ros_node(void)
{
    if (iniatialized) {
        init_options = rcl_get_zero_initialized_init_options();
        allocator = rcl_get_default_allocator();

        // Initialize micro-ROS with options
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            LOG_ERR("Failed to initialize init options");
        }

        if (rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID) != RCL_RET_OK) {
            LOG_ERR("Failed to set domain id");
        }
    }

    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)
            != RCL_RET_OK) {
        LOG_ERR("Failed to initialize support");
        return -1;
    }

    // Initialize node
    if (rclc_node_init_default(&node, CONFIG_ROS_NAMESPACE, "", &support) != RCL_RET_OK) {
        LOG_ERR("Failed to create node");
        return -1;
    }

    // Initialize publishers
    init_odometry_publisher(&node);
    init_scan_publisher(&node);
    init_cmd_vel_subscriber(&node);

    // Initialize executor
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator)) {
        LOG_ERR("Failed to initialize executor");
        return -1;
    }

    // Add subscriber to executor
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

    if (iniatialized) {
        // Initialize base interface
        base_callback.odometry_callback = send_odometry_callback;

        init_base_interface(&base_callback);

        // Initialize scan
        scan_callback.lidar_scan_callback = lidar_scan_callback;

        init_scan(&scan_callback);
    }

    iniatialized = false;
    state = WAITING_AGENT;

    return 0;
}

void spin_micro_ros_node(void)
{

    state = WAITING_AGENT;

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    LOG_WRN("Agent found");
                    state = AGENT_AVAILABLE;
                }
                break;
            case AGENT_AVAILABLE:
                if (init_micro_ros_node() == 0) {
                    LOG_WRN("Micro-ROS node initialized");
                    state = AGENT_CONNECTED;
                } else {
                    LOG_ERR("Failed to initialize micro-ROS node");
                    state = AGENT_DISCONNECTED;
                }
                break;
            case AGENT_CONNECTED:
                int ret = rmw_uros_ping_agent(1, 100);
                if (ret != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                }
                if (state == AGENT_CONNECTED) {
                    LOG_DBG("Spinning micro-ROS node");
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                }
                break;
            case AGENT_DISCONNECTED:
                LOG_WRN("Agent disconnected");
                destroy_micro_ros_node();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
        k_msleep(1000);
    }
}

K_THREAD_DEFINE(micro_ros_spin_thread, 8192, spin_micro_ros_node, NULL, NULL, NULL, 3, 0, 0);

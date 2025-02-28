#include <zephyr/kernel.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/string_utilities.h>
#include <microros_transports.h>
#include <rcl/init_options.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

#include <minipock_msgs/srv/get_chunk.h>
#include <minipock_msgs/srv/trig_update.h>
#include <type_utilities.h>

#include "base_interface.h"
#include "flash_storage.h"
#include "micro_ros_node.h"
#include "scan.h"
#include "update.h"
#include <autoconf.h>

LOG_MODULE_REGISTER(micro_ros_node, LOG_LEVEL_DBG);

#define UPDATE_CHUNK_SIZE 1024

static rclc_executor_t executor;
static rcl_publisher_t odom_publisher;
static rcl_publisher_t optic_odom_publisher;
static rcl_publisher_t scan_publisher;
static rcl_subscription_t cmd_vel_subscriber;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_init_options_t init_options;
static rcl_client_t client;
static rcl_client_t client_update;

static char namespace[50];

static struct base_interface_trigger base_callback;
static struct base_interface_trigger
        optic_sensor_callback; // Same struct as base_callback as they are both odometry
static struct scan_trigger scan_callback;

minipock_msgs__srv__TrigUpdate_Request req;
minipock_msgs__srv__TrigUpdate_Response res;

minipock_msgs__srv__GetChunk_Request req_chunk;
minipock_msgs__srv__GetChunk_Response res_chunk;

static geometry_msgs__msg__Twist cmd_vel_twist_msg;
static uint64_t ros_timestamp;

bool iniatialized = true;

static int chunk_id = 0;
static uint8_t version[3] = { 0, 0, 0 };

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

    char frame_id[75];
    snprintf(frame_id, sizeof(frame_id), "%s/lds_01_link", namespace);
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

// Send odometry coming from sparkfun paa5160e1
void send_optic_odometry_callback(float x, float y, float theta)
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

    char frame_id[60];
    snprintf(frame_id, sizeof(frame_id), "%s/odom_optc", namespace);
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
        if (rcl_publish(&optic_odom_publisher, &pose_stamped_msg, NULL) != RCL_RET_OK) {
            LOG_ERR("Failed to publish sensor odometry message");
        } else {
            LOG_DBG("Odometry message published");
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

    char frame_id[60];
    snprintf(frame_id, sizeof(frame_id), "%s/odom", namespace);
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
    char odom_topic_name[60];

    snprintf(odom_topic_name, sizeof(odom_topic_name), "/%s/odom_raw", namespace);
    rclc_publisher_init_best_effort(&odom_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            odom_topic_name);
}

void init_optic_odometry_publisher(rcl_node_t *node)
{
    char optic_odom_topic_name[65];

    snprintf(optic_odom_topic_name, sizeof(optic_odom_topic_name), "/%s/odom_optc_raw", namespace);
    rclc_publisher_init_best_effort(&optic_odom_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            optic_odom_topic_name);
}

void init_scan_publisher(rcl_node_t *node)
{
    char scan_topic_name[60];
    snprintf(scan_topic_name, sizeof(scan_topic_name), "/%s/scan_raw", namespace);
    static rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    rclc_publisher_init(&scan_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            scan_topic_name,
            &custom_qos_profile);
}

void init_cmd_vel_subscriber(rcl_node_t *node)
{
    char cmd_vel_topic_name[60];
    snprintf(cmd_vel_topic_name, sizeof(cmd_vel_topic_name), "/%s/cmd_vel", namespace);
    rclc_subscription_init_best_effort(&cmd_vel_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cmd_vel_topic_name);
}

void update_chunk_received(const void *msgin)
{
    const minipock_msgs__srv__GetChunk_Response *in
            = (const minipock_msgs__srv__GetChunk_Response *)msgin;

    LOG_DBG("Chunk received: %lld", in->chunk_id);
    LOG_DBG("Chunk Success: %d", in->success);
    LOG_DBG("Chunk Checksum: %lld", in->chunk_checksum);

    if (in->success == 0) {
        LOG_DBG("Message received %d, size: %d", chunk_id, in->chunk_byte.size);

        uint8_t crc = 0;
        crc = crc8_ccitt(crc, in->chunk_byte.data, in->chunk_byte.size);

        LOG_DBG("CRC: %d", crc);
        if (crc == in->chunk_checksum) {
            LOG_DBG("Checksum OK");
            update_write_chunk(chunk_id, in->chunk_byte.data, in->chunk_byte.size);
            chunk_id++;
        } else {
            LOG_ERR("Checksum failed");
        }

        minipock_msgs__srv__GetChunk_Request__init(&req_chunk);

        req_chunk.version.major = version[0];
        req_chunk.version.minor = version[1];
        req_chunk.version.patch = version[2];

        req_chunk.chunk_id = chunk_id;
        req_chunk.chunk_size = UPDATE_CHUNK_SIZE;

        int64_t seq;

        int ret = rcl_send_request(&client_update, &req_chunk, &seq);

        if (ret != RCL_RET_OK) {
            LOG_ERR("Failed to send request %d", ret);
            return;
        }
    } else if (in->success == 2) {
        LOG_INF("Download Finished");
        // boot_request_upgrade((int)BOOT_UPGRADE_PERMANENT);
    }
}

void update_service_callback(const void *msgin)
{
    const minipock_msgs__srv__TrigUpdate_Response *in
            = (const minipock_msgs__srv__TrigUpdate_Response *)msgin;

    if (in->success == 0 && in->new_version_available) {
        LOG_DBG("New version: %d.%d.%d",
                in->new_version.major,
                in->new_version.minor,
                in->new_version.patch);

        minipock_msgs__srv__GetChunk_Request__init(&req_chunk);

        req_chunk.version.major = in->new_version.major;
        req_chunk.version.minor = in->new_version.minor;
        req_chunk.version.patch = in->new_version.patch;

        version[0] = in->new_version.major;
        version[1] = in->new_version.minor;
        version[2] = in->new_version.patch;

        req_chunk.chunk_id = chunk_id;
        req_chunk.chunk_size = UPDATE_CHUNK_SIZE;

        int64_t seq;

        int ret = rcl_send_request(&client_update, &req_chunk, &seq);

        if (ret != RCL_RET_OK) {
            LOG_ERR("Failed to send request %d", ret);
            return;
        }
    }
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
    static zephyr_transport_params_t agent_param
            = { { 0, 0, 0 }, CONFIG_MICROROS_AGENT_IP, CONFIG_MICROROS_AGENT_PORT };
    // flash_storage_read("agent_ip", agent_param.ip, sizeof(agent_param.ip));
    memset(agent_param.ip, 0, sizeof(agent_param.ip));
    strcpy(agent_param.ip, "192.168.169.25");
    printk("Agent IP: %s\n", agent_param.ip);
    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)&agent_param,
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

    return 0;
}

int micro_ros_node_get_last_version(void)
{
    int64_t seq;
    int ret = 0;

    minipock_msgs__srv__TrigUpdate_Request__init(&req);

    uint8_t major, minor, revision;
    update_get_current_version(&major, &minor, &revision);

    req.actual_version.major = major;
    req.actual_version.minor = minor;
    req.actual_version.patch = revision;

    k_sleep(K_SECONDS(1));

    ret = rcl_send_request(&client, &req, &seq);
    if (ret != RCL_RET_OK) {
        LOG_ERR("Failed to send request");
    }

    return ret;
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

        // flash_storage_read(NAMESPACE, namespace, sizeof(namespace));
        memset(namespace, 0, sizeof(namespace));
        strcpy(namespace, "minipock_0");
        printk("Namespace: %s\n", namespace);

        if (strlen(namespace) == sizeof(namespace)) {
            strcpy(namespace, CONFIG_ROS_NAMESPACE);
            flash_storage_write(NAMESPACE, namespace, strlen(namespace));
        }
    }

    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)
            != RCL_RET_OK) {
        LOG_ERR("Failed to initialize support");
        return -1;
    }

    // Initialize node
    int ret;
    // print namespace
    printk("Namespace: %s\n", namespace);
    ret = rclc_node_init_default(&node, namespace, "", &support);
    if (ret != RCL_RET_OK) {
        printk("Failed to create node, ret: %d\n", ret);
        LOG_ERR("Failed to create node");
        return -1;
    }

    // Initialize publishers
    init_odometry_publisher(&node);
    init_optic_odometry_publisher(&node);
    init_scan_publisher(&node);
    init_cmd_vel_subscriber(&node);

    // Create client
    char service_name[75];
    snprintf(service_name, sizeof(service_name), "/%s/firmware_update", namespace);
    if (rclc_client_init_default(&client,
                &node,
                ROSIDL_GET_SRV_TYPE_SUPPORT(minipock_msgs, srv, TrigUpdate),
                service_name)
            != RCL_RET_OK) {
        LOG_ERR("Failed to create client");
        return -1;
    }

    snprintf(service_name, sizeof(service_name), "/%s/firmware_update/chunk", namespace);
    if (rclc_client_init_default(&client_update,
                &node,
                ROSIDL_GET_SRV_TYPE_SUPPORT(minipock_msgs, srv, GetChunk),
                service_name)
            != RCL_RET_OK) {
        LOG_ERR("Failed to create client");
        return;
    }

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 3, &allocator)) {
        LOG_ERR("Failed to initialize executor");
        return -1;
    }

    // Add subscribers to executor
    if (rclc_executor_add_subscription(&executor,
                &cmd_vel_subscriber,
                &cmd_vel_twist_msg,
                &subscription_callback,
                ON_NEW_DATA)) {
        LOG_ERR("Failed to add subscription to executor");
        return -1;
    }

    // Add client to executor
    if (rclc_executor_add_client(&executor, &client, &res, update_service_callback) != RCL_RET_OK) {
        LOG_ERR("Failed to add client to executor");
        return -1;
    }

    // Initialize chunk response
    static micro_ros_utilities_memory_conf_t conf = { 0 };

    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = UPDATE_CHUNK_SIZE;
    conf.max_basic_type_sequence_capacity = UPDATE_CHUNK_SIZE;

    bool success = micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(minipock_msgs, srv, GetChunk_Response), &res_chunk, conf);

    if (rclc_executor_add_client(&executor, &client_update, &res_chunk, update_chunk_received)
            != RCL_RET_OK) {
        LOG_ERR("Failed to add client to executor");
        return;
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

        // Initialize sensor
        optic_sensor_callback.odometry_callback = send_optic_odometry_callback;

        init_optic_sensor(&optic_sensor_callback);
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

                    micro_ros_node_get_last_version();

                    state = AGENT_CONNECTED;
                } else {
                    LOG_ERR("Failed to initialize micro-ROS node");
                    state = AGENT_DISCONNECTED;
                }
                break;
            case AGENT_CONNECTED:
                int ret = rmw_uros_ping_agent(100, 10);
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
                // k_timer_stop(&my_timer);
                destroy_micro_ros_node();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
        k_msleep(100);
    }
}

K_THREAD_DEFINE(micro_ros_spin_thread, 8192, spin_micro_ros_node, NULL, NULL, NULL, 3, 0, 0);

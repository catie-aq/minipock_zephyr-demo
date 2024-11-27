#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <microros_transports.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "base_interface.h"
#include "common.h"

// Wireless management
static struct net_mgmt_event_callback wifi_shell_mgmt_cb;

#define RCCHECK(fn)                                                                                \
    {                                                                                              \
        rcl_ret_t temp_rc = fn;                                                                    \
        if ((temp_rc != RCL_RET_OK)) {                                                             \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);           \
            return 1;                                                                              \
        }                                                                                          \
    }
#define RCSOFTCHECK(fn)                                                                            \
    {                                                                                              \
        rcl_ret_t temp_rc = fn;                                                                    \
        if ((temp_rc != RCL_RET_OK)) {                                                             \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);         \
        }                                                                                          \
    }

rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_twist_msg;

// LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Network
#define DHCP_OPTION_NTP (42)
static uint8_t ntp_server[4];

static struct net_mgmt_event_callback mgmt_cb;
static struct net_dhcpv4_option_callback dhcp_cb;

rclc_executor_t executor;

static bool connected = 0;

static void wifi_mgmt_event_handler(
        struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    if (NET_EVENT_IPV4_ADDR_ADD == mgmt_event) {
        printk("IPv4 address added\n");
        connected = 1;
    }
}

void subscription_callback(const void *msgin)
{
    gpio_pin_toggle_dt(&led);

    const geometry_msgs__msg__Twist *uros_msg = (const geometry_msgs__msg__Twist *)msgin;

    send_command(uros_msg->linear.x,
            uros_msg->linear.y,
            uros_msg->linear.z,
            uros_msg->angular.x,
            uros_msg->angular.y,
            uros_msg->angular.z);
}

static void start_dhcpv4_client(struct net_if *iface, void *user_data)
{
    ARG_UNUSED(user_data);

    printk("Start on %s: index=%d", net_if_get_device(iface)->name, net_if_get_by_iface(iface));
    net_dhcpv4_start(iface);
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

static void handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    int i = 0;

    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
        char buf[NET_IPV4_ADDR_LEN];

        if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
            continue;
        }

        printk("   Address[%d]: %s",
                net_if_get_by_iface(iface),
                net_addr_ntop(AF_INET,
                        &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
                        buf,
                        sizeof(buf)));
        printk("    Subnet[%d]: %s",
                net_if_get_by_iface(iface),
                net_addr_ntop(
                        AF_INET, &iface->config.ip.ipv4->unicast[i].netmask, buf, sizeof(buf)));
        printk("    Router[%d]: %s",
                net_if_get_by_iface(iface),
                net_addr_ntop(AF_INET, &iface->config.ip.ipv4->gw, buf, sizeof(buf)));
        printk("Lease time[%d]: %u seconds",
                net_if_get_by_iface(iface),
                iface->config.dhcpv4.lease_time);
    }

    connected = 1;
}

static void option_handler(struct net_dhcpv4_option_callback *cb,
        size_t length,
        enum net_dhcpv4_msg_type msg_type,
        struct net_if *iface)
{
    char buf[NET_IPV4_ADDR_LEN];

    printk("DHCP Option %d: %s", cb->option, net_addr_ntop(AF_INET, cb->data, buf, sizeof(buf)));
}

int main()
{
    int ret;

    // Init LED0
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // ------ Wifi Configuration ------
    net_mgmt_init_event_callback(
            &wifi_shell_mgmt_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

    k_sleep(K_SECONDS(5));

    static struct wifi_connect_req_params wifi_args;

    wifi_args.security = WIFI_SECURITY_TYPE_PSK;
    wifi_args.channel = 13;
    wifi_args.psk = "";
    wifi_args.psk_length = strlen(wifi_args.psk);
    wifi_args.ssid = "";
    wifi_args.ssid_length = strlen(wifi_args.ssid);
    wifi_args.timeout = 0;

    struct net_if *iface = net_if_get_wifi_sta();

    if (!iface) {
        printf("No default network interface\n");
    }

    ret = net_mgmt(
            NET_REQUEST_WIFI_CONNECT, iface, &wifi_args, sizeof(struct wifi_connect_req_params));

    if (ret < 0) {
        printf("Connection request failed %d\n", ret);
    } else {
        printf("Connection requested\n");
    }

    while (!connected) {
        // printf("Waiting for connection\n");
        usleep(10000);
    }
    printf("Connection OK\n");

    k_sleep(K_SECONDS(5));

    // Init micro-ROS
    static zephyr_transport_params_t agent_param = { { 0, 0, 0 }, "192.168.1.2", "8888" };

    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)&agent_param,
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        printk("Waiting for agent...\n");
        k_sleep(K_SECONDS(1)); // Sleep for 1 second
    }

    agent_connected = true;

    printk("\nAgent found!\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID));

    // Initialize micro-ROS with options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, CONFIG_ROS_NAMESPACE, "", &support));

    // Create cmd vel subscriber
    char cmd_vel_topic_name[50];
    snprintf(cmd_vel_topic_name, sizeof(cmd_vel_topic_name), "/%s/cmd_vel", CONFIG_ROS_NAMESPACE);
    rclc_subscription_init_best_effort(&cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cmd_vel_topic_name);

    if (ret != RCL_RET_OK) {
        printk("Failed to create subscriber: %d\n", ret);
    }

    // Create odometry publisher
    char odom_topic_name[50];
    snprintf(odom_topic_name, sizeof(odom_topic_name), "/%s/odom_raw", CONFIG_ROS_NAMESPACE);
    ret = rclc_publisher_init_best_effort(&odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            odom_topic_name);

    if (ret != RCL_RET_OK) {
        printk("Failed to create odom publisher: %d\n", ret);
    }

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor,
            &cmd_vel_subscriber,
            &cmd_vel_twist_msg,
            &subscription_callback,
            ON_NEW_DATA));

    // Synchronize time
    RCCHECK(rmw_uros_sync_session(1000));
    ros_timestamp = rmw_uros_epoch_millis() - k_uptime_get();

    // Initialize motor interface
    struct base_interface_callbacks base_callback;
    base_callback.send_odometry_callback = send_odometry_callback;

    init_base_interface(&base_callback);

    // Start micro-ROS thread
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }
}

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <microros_transports.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "pb_decode.h"
#include "pb_encode.h"
#include "src/minipock.pb.h"

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
rcl_publisher_t scan_publisher;
geometry_msgs__msg__Twist cmd_vel_twist_msg;

// LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// UART
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(rbdc_serial_port));

// RX UART buffer for odom message
static char rx_buf[odom_size];
static int rx_buf_pos;

K_MSGQ_DEFINE(uart_msgq, odom_size, 10, 4);

// Save timestamp of last message
static uint64_t last_msg_timestamp;
static uint64_t ros_timestamp;

// Network
#define DHCP_OPTION_NTP (42)
static uint8_t ntp_server[4];

static struct net_mgmt_event_callback mgmt_cb;
static struct net_dhcpv4_option_callback dhcp_cb;

rclc_executor_t executor;

static bool connected = 0;
static bool agent_connected = 0;

// LiDAR
struct lidar_data {
    float ranges[12];
    float intensities[12];
    float start_angle;
    float end_angle;
};

#define NB_MSG 11
#define NB_POINTS_PER_MSG 12
#define NB_POINTS (NB_MSG * NB_POINTS_PER_MSG)

static struct lidar_data lidar_msg[NB_MSG];
int lidar_msg_index = 0;

static const struct device *sensor;

K_MSGQ_DEFINE(lidar_msgq, sizeof(struct lidar_data), 20, 4);

void process_odometry_msg()
{
    static char data[odom_size];

    while (k_msgq_get(&uart_msgq, &data, K_FOREVER) == 0) {

        pb_istream_t stream = pb_istream_from_buffer(data, odom_size);

        odom msg = odom_init_zero;

        if (!pb_decode(&stream, odom_fields, &msg)) {
            continue;
        }

        // Convert x, y, theta to quaternion
        float cy = cos((float)msg.theta * 0.5);
        float sy = sin((float)msg.theta * 0.5);
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

        pose_stamped_msg.header.frame_id.data = "odom";

        pose_stamped_msg.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
        pose_stamped_msg.header.stamp.nanosec
                = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

        pose_stamped_msg.pose.position.x = (float)msg.x;
        pose_stamped_msg.pose.position.y = (float)msg.y;
        pose_stamped_msg.pose.position.z = 0;

        pose_stamped_msg.pose.orientation.w = q0;
        pose_stamped_msg.pose.orientation.x = q1;
        pose_stamped_msg.pose.orientation.y = q2;
        pose_stamped_msg.pose.orientation.z = q3;

        if (rcl_publish(&odom_publisher, &pose_stamped_msg, NULL) != RCL_RET_OK) {
            printf("Failed to publish odometry message\n");
        }
    }
}

void uart_callback_handler(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

        if (k_uptime_get() - last_msg_timestamp > 10) {
            rx_buf_pos = 0;
            k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
        }

        if (uart_irq_rx_ready(dev)) {
            int length = uart_fifo_read(dev, &rx_buf[rx_buf_pos], 1);
            rx_buf_pos += length;
            last_msg_timestamp = k_uptime_get();
        }
    }
}

void subscription_callback(const void *msgin)
{
    gpio_pin_toggle_dt(&led);

    const geometry_msgs__msg__Twist *uros_msg = (const geometry_msgs__msg__Twist *)msgin;

    cmd_vel msg = cmd_vel_init_zero;
    msg.linear_x = (float)uros_msg->linear.x;
    msg.linear_y = (float)uros_msg->linear.y;
    msg.linear_z = (float)uros_msg->linear.z;
    msg.angular_x = (float)uros_msg->angular.x;
    msg.angular_y = (float)uros_msg->angular.y;
    msg.angular_z = (float)uros_msg->angular.z;

    uint8_t buffer[cmd_vel_size];

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&stream, cmd_vel_fields, &msg)) {
        return;
    }

    // If the length is still 0, manually add a dummy field
    if (stream.bytes_written == 0) {
        buffer[0] = 0x00; // Add a dummy byte
        stream.bytes_written = 1;
    }

    for (int i = 0; i < stream.bytes_written; i++) {
        uart_poll_out(uart_dev, buffer[i]);
    }
}

static void start_dhcpv4_client(struct net_if *iface, void *user_data)
{
    ARG_UNUSED(user_data);

    printk("Start on %s: index=%d", net_if_get_device(iface)->name, net_if_get_by_iface(iface));
    net_dhcpv4_start(iface);
}

static void handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    int i = 0;

    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
        char buf[NET_IPV4_ADDR_LEN];

        if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP) {
            continue;
        }

        printk("   Address[%d]: %s",
                net_if_get_by_iface(iface),
                net_addr_ntop(AF_INET,
                        &iface->config.ip.ipv4->unicast[i].address.in_addr,
                        buf,
                        sizeof(buf)));
        printk("    Subnet[%d]: %s",
                net_if_get_by_iface(iface),
                net_addr_ntop(AF_INET, &iface->config.ip.ipv4->netmask, buf, sizeof(buf)));
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

            scan.header.frame_id.data = "lds_01_link";
            scan.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
            scan.header.stamp.nanosec
                    = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

            float angle_increment = 0;
            if (lidar_msg[0].start_angle > lidar_msg[NB_MSG - 1].end_angle) {
                angle_increment
                        = (6.28 - lidar_msg[0].start_angle + lidar_msg[NB_MSG - 1].end_angle)
                        / (float)NB_POINTS;
            } else {
                angle_increment = (lidar_msg[NB_MSG - 1].end_angle - lidar_msg[0].start_angle)
                        / (float)NB_POINTS;
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
                // blink LED
                gpio_pin_toggle_dt(&led);
                int ret = rcl_publish(&scan_publisher, &scan, NULL);
                if (ret != RCL_RET_OK) {
                    printk("Failed to publish message\n");
                }
            }
        }
    }
}

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
    struct sensor_value val[15];

    struct lidar_data lidar_data;

    sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, val);

    lidar_data.start_angle = val[1].val1 / 100 * 3.14 / 180;
    lidar_data.end_angle = val[1].val2 / 100 * 3.14 / 180;

    for (int i = 0; i < NB_POINTS_PER_MSG; i++) {
        lidar_data.ranges[i] = (float)val[i + 2].val1 / 1000.;
        lidar_data.intensities[i] = (float)val[i + 2].val2;
    }

    while (k_msgq_put(&lidar_msgq, &lidar_data, K_NO_WAIT) != 0) {
        printk("Failed to put data in queue\n");
        k_msgq_purge(&lidar_msgq);
    }
}

#define STACKSIZE 8162
K_THREAD_STACK_DEFINE(thread_stack, STACKSIZE);
struct k_thread thread_data;

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

    // Initialize LiDAR
    sensor = DEVICE_DT_GET(DT_NODELABEL(lidar0));
    if (!device_is_ready(sensor)) {
        printk("Sensor not ready\n");
        return 0;
    }

    static struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_DISTANCE,
    };

    sensor_trigger_set(sensor, &trig, trigger_handler);

    // Init UART
    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return 0;
    }

    // Init micro-ROS
    static zephyr_transport_params_t agent_param = { { 0, 0, 0 }, "192.168.1.4", "8888" };

    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)&agent_param,
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

    net_mgmt_init_event_callback(&mgmt_cb, handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&mgmt_cb);

    net_dhcpv4_init_option_callback(
            &dhcp_cb, option_handler, DHCP_OPTION_NTP, ntp_server, sizeof(ntp_server));

    net_dhcpv4_add_option_callback(&dhcp_cb);

    net_if_foreach(start_dhcpv4_client, NULL);

    while (!connected) {
        // printf("Waiting for connection\n");
        usleep(10000);
    }
    printf("Connection OK\n");

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        printk("Waiting for agent...\n");
        k_sleep(K_SECONDS(1)); // Sleep for 1 second
    }

    agent_connected = true;

    printk("Agent found!\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID));

    // Initialize micro-ROS with options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "zephyr", "", &support));

    // Create cmd vel subscriber
    RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"));

    // Create odometry publisher
    RCCHECK(rclc_publisher_init_best_effort(&odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            "/odom_raw"));

    // Create scan publisher
    static rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    rclc_publisher_init(&scan_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "/scan",
            &custom_qos_profile);

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

    // Set UART callback
    ret = uart_irq_callback_set(uart_dev, uart_callback_handler);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }

    // Enable UART RX interrupt
    uart_irq_rx_enable(uart_dev);
    uart_irq_tx_disable(uart_dev);

    int priority = 5;
    k_thread_create(&thread_data,
            thread_stack,
            K_THREAD_STACK_SIZEOF(thread_stack),
            send_lidar_data,
            NULL,
            NULL,
            NULL,
            priority,
            0,
            K_NO_WAIT);

    // Start micro-ROS thread
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }
}

K_THREAD_DEFINE(uart_rx_thread, 1024, process_odometry_msg, NULL, NULL, NULL, 5, 0, 0);

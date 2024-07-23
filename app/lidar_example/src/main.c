/*
 * Copyright (c) 2023 CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rmw_microros/rmw_microros.h>

#include <microros_transports.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#include "pb_decode.h"
#include "pb_encode.h"

#include <zephyr/net/net_context.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_SENSOR_LOG_LEVEL);

#define DHCP_OPTION_NTP (42)

static uint8_t ntp_server[4];

static struct net_mgmt_event_callback mgmt_cb;

static struct net_dhcpv4_option_callback dhcp_cb;

static const struct device *sensor;

static rclc_executor_t executor;

#define STACKSIZE 8162
K_THREAD_STACK_DEFINE(thread_stack, STACKSIZE);
struct k_thread thread_data;

void uros_transport_task();

K_THREAD_DEFINE(uros_transport, 2048, uros_transport_task, NULL, NULL, NULL, 7, 0, 0);

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

rcl_publisher_t scan_publisher;
rcl_subscription_t odom_subscriber;
geometry_msgs__msg__Twist cmd_vel_twist_msg;

// LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Save timestamp of last message
static uint64_t last_msg_timestamp;
static uint64_t ros_timestamp;

// Network management
static bool connected = 0;
static volatile bool agent_connected = false;

// Struct contain ranges, intensities, start_angle, end_angle
static struct lidar_data {
    float ranges[12];
    float intensities[12];
    float start_angle;
    float end_angle;
};

K_MSGQ_DEFINE(lidar_msgq, sizeof(struct lidar_data), 100, 4);

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

#define NB_MSG 11
#define NB_POINTS_PER_MSG 12
#define NB_POINTS (NB_MSG * NB_POINTS_PER_MSG)

static struct lidar_data lidar_msg[NB_MSG];
int lidar_msg_index = 0;

extern void send_lidar_data(void *, void *, void *);

void send_lidar_data(void *, void *, void *)
{
    static float range_to_send[NB_POINTS];
    static float intensity_to_send[NB_POINTS];
    // float start_angle_to_send, end_angle_to_send;

    struct lidar_data lidar_data;

    static volatile sensor_msgs__msg__LaserScan scan;

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

void uros_transport_task()
{
    while (1) {
        if (agent_connected) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            k_msleep(1000);
        } else {
            k_msleep(1000);
        }
    }
}

static void trigger_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    // Blink LED

    struct sensor_value val[15];

    static int index = 0;
    static bool flag = false;

    struct lidar_data lidar_data;

    sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, val);

    // printk("Start angle: %d, End angle: %d\n", val[1].val1, val[1].val2);

    // if (index >= 120) {
    //     flag = false;
    //     index = 0;

    //     lidar_data->end_angle = 13.; //val[1].val2 / 100 * 3.14 / 180;

    //     // if(k_uptime_get() - last_msg_timestamp < 10) {
    //     //     return;
    //     // }
    //     // send_lidar_data();
    //     // k_mutex_lock(&buffer_mutex, K_FOREVER);
    //     // k_work_submit(&lidar_send_data_work);

    //     // k_work_submit_to_queue(&my_work_q, &lidar_send_data_work);
    //     while (k_msgq_put(&lidar_msgq, lidar_data, K_NO_WAIT) != 0) {
    //         printk("Failed to put data in queue\n");
    //         k_msgq_purge(&lidar_msgq);
    //     }
    //     // k_msgq_put(&lidar_msgq, &lidar_data, K_NO_WAIT);

    //     // last_msg_timestamp = k_uptime_get();
    //     // rcl_publish(&scan_publisher, &scan, NULL);
    // } else {
    //     if (!flag) {
    //         lidar_data->start_angle = val[1].val1 / 100 * 3.14 / 180;
    //         flag = true;
    //     }

    //     for (int i = 0; i < 12; i++) {
    //         lidar_data->ranges[index + i] = 12.;//(float)val[i + 2].val1 / 100.;
    //         lidar_data->intensities[index + i] = 12.;//(float)val[i + 2].val2 / 100.;
    //         index++;
    //     }
    // }

    lidar_data.start_angle = val[1].val1 / 100 * 3.14 / 180;
    lidar_data.end_angle = val[1].val2 / 100 * 3.14 / 180;

    for (int i = 0; i < NB_POINTS_PER_MSG; i++) {
        lidar_data.ranges[i] = (float)val[i + 2].val1 / 100.;
        lidar_data.intensities[i] = (float)val[i + 2].val2 / 100.;
    }

    while (k_msgq_put(&lidar_msgq, &lidar_data, K_NO_WAIT) != 0) {
        printk("Failed to put data in queue\n");
        k_msgq_purge(&lidar_msgq);
    }
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // Toggles LED0
    gpio_pin_toggle_dt(&led);

    struct sensor_value val[15];

    // sensor_sample_fetch(sensor);
    sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, val);

    // static sensor_msgs__msg__LaserScan scan;

    // scan.header.frame_id.data = "lds_01_link";

    // scan.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
    // scan.header.stamp.nanosec = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

    // scan.angle_min = val[1].val1 / 100 * 3.14 / 180;
    // scan.angle_max = val[1].val2 / 100 * 3.14 / 180;

    // scan.angle_increment = 0.8 * 3.14 / 180;
    // scan.time_increment = 0.1 / 540;
    // scan.scan_time = 0.1;
    // scan.range_min = 0.02;
    // scan.range_max = 12;

    // for (int i = 0; i < 12; i++) {
    //     ranges[i] = (float)val[i + 2].val1 / 100.;
    //     intensities[i] = (float)val[i + 2].val2 / 100.;
    // }

    // start_angle = val[1].val1 / 100 * 3.14 / 180;
    // end_angle = val[1].val2 / 100 * 3.14 / 180;

    // k_work_submit(&lidar_send_data_work);

    // scan.ranges.data = ranges;
    // scan.intensities.data = intensities;
    // scan.ranges.size = 12;
    // scan.intensities.size = 12;

    // RCSOFTCHECK(rcl_publish(&scan_publisher, &scan, NULL));
}

int main()
{
    int ret;

    printk("Starting micro-ROS Zephyr app\n");

    int priority = 5; // Set your thread's priority
    k_tid_t thread_id = k_thread_create(&thread_data,
            thread_stack,
            K_THREAD_STACK_SIZEOF(thread_stack),
            send_lidar_data,
            NULL,
            NULL,
            NULL,
            priority,
            0,
            K_NO_WAIT);

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
        LOG_ERR("Sensor not ready");
        return 0;
    }

    static struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_DISTANCE,
    };

    sensor_trigger_set(sensor, &trig, trigger_handler);

    // Init micro-ROS
    static zephyr_transport_params_t agent_param = { { 0, 0, 0 }, "192.168.1.4", "8888" };

    // Init micro-ROS
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

    // Create scan publisher
    static rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    rclc_publisher_init(&scan_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "scan",
            &custom_qos_profile);

    // create timer
    rcl_timer_t timer;
    // const unsigned int timer_timeout = 10;
    // RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
    // timer_callback));

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    // RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Synchronize time
    RCCHECK(rmw_uros_sync_session(1000));
    ros_timestamp = rmw_uros_epoch_millis();

    while (1) {
        k_msleep(1000);
    }

    // rclc_executor_spin(&executor);
}

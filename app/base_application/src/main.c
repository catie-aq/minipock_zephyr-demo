#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <microros_transports.h>
#include <rmw_microros/rmw_microros.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

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

rcl_subscription_t subscriber;
rcl_publisher_t publisher;

geometry_msgs__msg__Twist msg;

// LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// UART
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(rbdc_serial_port));

// RX UART Odom message
static char rx_buf[odom_size];
static int rx_buf_pos;

K_MSGQ_DEFINE(uart_msgq, odom_size, 10, 4);

// Save timestamp of last message
static uint64_t last_msg_timestamp;
static uint64_t ros_timestamp;

void process_odometry_msg()
{
    static char data[odom_size];

    while (k_msgq_get(&uart_msgq, &data, K_FOREVER) == 0) {
        // while (1) {
        gpio_pin_toggle_dt(&led);

        pb_istream_t stream = pb_istream_from_buffer(data, odom_size);

        odom msg = odom_init_zero;

        if (!pb_decode(&stream, odom_fields, &msg)) {
            continue;
        }

        // Convert x, y, theta to quaternion
        float q0 = cos((float)msg.theta / 2);
        float q1 = 0;
        float q2 = 0;
        float q3 = sin((float)msg.theta / 2);

        // Convert to ROS message
        nav_msgs__msg__Odometry odometry_msg;

        odometry_msg.header.frame_id.data = "odom";
        odometry_msg.child_frame_id.data = "base_link";

        odometry_msg.header.stamp.sec = (int32_t)((ros_timestamp + k_uptime_get()) / 1000);
        odometry_msg.header.stamp.nanosec
                = (uint32_t)((ros_timestamp + k_uptime_get()) % 1000) * 1000000;

        odometry_msg.pose.pose.position.x = (float)msg.x;
        odometry_msg.pose.pose.position.y = (float)msg.y;
        odometry_msg.pose.pose.position.z = 0;

        odometry_msg.pose.pose.orientation.x = q0;
        odometry_msg.pose.pose.orientation.y = q1;
        odometry_msg.pose.pose.orientation.z = q2;
        odometry_msg.pose.pose.orientation.w = q3;

        odometry_msg.twist.twist.linear.x = 0;
        odometry_msg.twist.twist.linear.y = 0;
        odometry_msg.twist.twist.linear.z = 0;

        odometry_msg.twist.twist.angular.x = 0;
        odometry_msg.twist.twist.angular.y = 0;
        odometry_msg.twist.twist.angular.z = 0;

        // Set covariance to 0
        for (int i = 0; i < 36; i++) {
            odometry_msg.pose.covariance[i] = 0;
            odometry_msg.twist.covariance[i] = 0;
        }

        // Publish message
        rcl_publish(&publisher, &odometry_msg, NULL);
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

    // Init UART
    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return 0;
    }

    // Init micro-ROS
    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)DEVICE_DT_GET(DT_ALIAS(uros_serial_port)),
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        printk("Waiting for agent...\n");
        k_sleep(K_SECONDS(1)); // Sleep for 1 second
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, CONFIG_ROS_ROS_DOMAIN_ID));

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "zephyr", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
            &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // Create publisher
    RCCHECK(rclc_publisher_init_default(
            &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
            &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Synchronize time
    RCCHECK(rmw_uros_sync_session(1000));
    ros_timestamp = rmw_uros_epoch_millis();

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

    // Enble UART RX interrupts
    uart_irq_rx_enable(uart_dev);
    uart_irq_tx_disable(uart_dev);

    rclc_executor_spin(&executor);

    while (1) {
        rclc_executor_spin_some(&executor, 100);
        usleep(100000);
    }

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
}

K_THREAD_DEFINE(uart_rx_thread, 1024, process_odometry_msg, NULL, NULL, NULL, 5, 0, 0);
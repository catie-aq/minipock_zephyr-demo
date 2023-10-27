#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

#include <stdio.h>
#include <unistd.h>

#include <microros_transports.h>
#include <rmw_microros/rmw_microros.h>

#include <zephyr/drivers/gpio.h>

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
geometry_msgs__msg__Twist msg;

// Inti LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    // printf("Received: x: %f, y: %f, z: %f\n", msg->linear.x, msg->linear.y, msg->linear.z);
    // printf("Received: x: %f, y: %f, z: %f\n", msg->angular.x, msg->angular.y, msg->angular.z);

    // Toggle LED0
    gpio_pin_toggle_dt(&led);
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

    // Init micro-ROS
    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
            (void *)DEVICE_DT_GET(DT_ALIAS(uros_serial_port)),
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "zephyr", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
            &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
            &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    rclc_executor_spin(&executor);

    while (1) {
        rclc_executor_spin_some(&executor, 100);
        usleep(100000);
    }

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
}
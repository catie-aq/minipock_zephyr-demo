#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pb_decode.h"
#include "pb_encode.h"

#include "base_interface.h"
#include "src/minipock.pb.h"

LOG_MODULE_REGISTER(base_interface, LOG_LEVEL_INF);

static struct base_interface_callbacks *base_interface_callbacks;

// Save timestamp of last message
static uint64_t last_msg_timestamp;

// UART
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(rbdc_serial_port));

// RX UART buffer for odom message
static char rx_buf[odom_size];
static int rx_buf_pos;

K_MSGQ_DEFINE(uart_msgq, odom_size, 10, 4);

void process_odometry_msg()
{
    static char data[odom_size];

    while (k_msgq_get(&uart_msgq, &data, K_FOREVER) == 0) {

        pb_istream_t stream = pb_istream_from_buffer(data, odom_size);

        odom msg = odom_init_zero;

        if (!pb_decode(&stream, odom_fields, &msg)) {
            continue;
        }

        if (base_interface_callbacks->send_odometry_callback != NULL) {
            base_interface_callbacks->send_odometry_callback(
                    (float)msg.x, (float)msg.y, (float)msg.theta);
        }
    }
}

void uart_callback_handler(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

        if (k_uptime_get() - last_msg_timestamp > 5) {
            if (rx_buf_pos == odom_size) {
                k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
            }
            rx_buf_pos = 0;
        }

        if (uart_irq_rx_ready(dev)) {
            int length = uart_fifo_read(dev, &rx_buf[rx_buf_pos], 1);
            rx_buf_pos += length;
            last_msg_timestamp = k_uptime_get();
        }
    }
}

void send_command(float linear_x,
        float linear_y,
        float linear_z,
        float angular_x,
        float angular_y,
        float angular_z)
{
    cmd_vel msg = cmd_vel_init_zero;
    msg.linear_x = linear_x;
    msg.linear_y = linear_y;
    msg.linear_z = linear_z;
    msg.angular_x = angular_x;
    msg.angular_y = angular_y;
    msg.angular_z = angular_z;

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

int init_base_interface(struct base_interface_callbacks *callbacks)
{
    int ret;

    base_interface_callbacks = callbacks;

    // Init UART
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not found!");
        return -ENODEV;
    }

    ret = uart_irq_callback_set(uart_dev, uart_callback_handler);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            LOG_ERR("Interrupt-driven UART API support not enabled");
        } else if (ret == -ENOSYS) {
            LOG_ERR("UART device does not support interrupt-driven API");
        } else {
            LOG_ERR("Error setting UART callback: %d", ret);
        }
        return ret;
    }

    // Enable UART RX interrupt
    uart_irq_rx_enable(uart_dev);
    uart_irq_tx_disable(uart_dev);

    return 0;
}

K_THREAD_DEFINE(uart_rx_thread, 1024, process_odometry_msg, NULL, NULL, NULL, 5, 0, 0);

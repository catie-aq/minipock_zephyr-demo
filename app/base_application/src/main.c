#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "common.h"
#include "flash_storage.h"
#include "micro_ros_node.h"
#include "update.h"

// Wireless management
static struct net_mgmt_event_callback wifi_shell_mgmt_cb;

// LED0
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static bool connected = 0;

static void wifi_mgmt_event_handler(
        struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    if (NET_EVENT_IPV4_ADDR_ADD == mgmt_event) {
        printk("IPv4 address added\n");
        connected = 1;
    }
}

int main()
{
    int ret;
    uint8_t major, minor, revision;
    char ssid[32];
    char password[32];
    uint8_t channel;

    update_get_current_version(&major, &minor, &revision);

    printk("MiniPock version: %d.%d.%d\n", major, minor, revision);

    // Init LED0
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // Init Flash Storage
    ret = flash_storage_init();
    if (ret < 0) {
        printf("Flash storage init failed\n");
        return 0;
    }

    // Read SSID and Password from Flash
    flash_storage_read(SSID, ssid, sizeof(ssid));
    flash_storage_read(PASSWORD, password, sizeof(password));
    flash_storage_read(CHANNEL, &channel, sizeof(channel));

    // ------ Wifi Configuration ------
    net_mgmt_init_event_callback(
            &wifi_shell_mgmt_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

    k_sleep(K_SECONDS(5));

    static struct wifi_connect_req_params wifi_args;

    wifi_args.security = WIFI_SECURITY_TYPE_PSK;
    wifi_args.channel = channel;
    wifi_args.psk = CONFIG_MICROROS_WIFI_PASSWORD;
    wifi_args.psk_length = strlen(wifi_args.psk);
    wifi_args.ssid = CONFIG_MICROROS_WIFI_SSID;
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
        k_usleep(10000);
    }
    printf("Connection OK\n");

    init_micro_ros_transport();

    while (1) {
        gpio_pin_toggle_dt(&led);
        k_sleep(K_MSEC(1000));
    }
}

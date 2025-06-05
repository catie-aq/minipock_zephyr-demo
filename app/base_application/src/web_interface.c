/*
 * Copyright (c) 2025, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>

#include "zephyr/device.h"
#include "zephyr/sys/util.h"
#include <zephyr/data/json.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util_macro.h>

#include "flash_storage.h"
#include "micro_ros_node.h"
#include "update.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(web_interface, LOG_LEVEL_DBG);

static uint8_t index_html_gz[] = {
#include "index.html.gz.inc"
};

static uint8_t main_js_gz[] = {
#include "main.js.gz.inc"
};

static uint8_t style_css_gz[] = {
#include "style.css.gz.inc"
};

static struct http_resource_detail_static index_html_gz_resource_detail = {
        .common =
            {
                .type = HTTP_RESOURCE_TYPE_STATIC,
                .bitmask_of_supported_http_methods = BIT(HTTP_GET),
                .content_encoding = "gzip",
                .content_type = "text/html",
            },
        .static_data = index_html_gz,
        .static_data_len = sizeof(index_html_gz),
    };

static struct http_resource_detail_static main_js_gz_resource_detail = {
        .common =
            {
                .type = HTTP_RESOURCE_TYPE_STATIC,
                .bitmask_of_supported_http_methods = BIT(HTTP_GET),
                .content_encoding = "gzip",
                .content_type = "text/javascript",
            },
        .static_data = main_js_gz,
        .static_data_len = sizeof(main_js_gz),
    };

static struct http_resource_detail_static style_css_gz_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_STATIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
			.content_encoding = "gzip",
			.content_type = "text/css",
		},
	.static_data = style_css_gz,
	.static_data_len = sizeof(style_css_gz),
};

static uint8_t uptime_buf[sizeof(STRINGIFY(INT64_MAX))];
static uint8_t version_buf[sizeof("255.255.255")];
static uint8_t ssid_buf[100];
static uint8_t update_network_buf[256];
static uint8_t update_ros_settings_buf[256];
static uint8_t ip_address_buf[256];
static uint8_t namespace_buf[256];
static uint8_t domain_id_buf[256];
static uint8_t micro_ros_status_buf[256];
static uint8_t estop_buf[256];
static uint8_t factory_reset_buf[256];
static uint8_t agent_ip_buf[256];

static int uptime_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;
            return snprintf(buffer, sizeof(uptime_buf), "%" PRId64, k_uptime_get());
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int version_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            uint8_t major, minor, revision;

            update_get_current_version(&major, &minor, &revision);

            return snprintf(buffer,
                    sizeof(uptime_buf),
                    "{\"version\":\"%d.%d.%d\"}",
                    major,
                    minor,
                    revision);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int ssid_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            char ssid[32];
            flash_storage_read(SSID, ssid, sizeof(ssid));

            return snprintf(buffer, sizeof(ssid_buf), "{\"ssid\":\"%s\"}", ssid);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int update_network_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (len > 0) {
                struct wifi_settings_struct {
                    char *ssid;
                    char *password;
                };

                struct wifi_settings_struct wifi_settings = { 0 };

                // Parse JSON data and update network settings
                struct json_obj_descr descr[] = {
                    JSON_OBJ_DESCR_PRIM(struct wifi_settings_struct, ssid, JSON_TOK_STRING),
                    JSON_OBJ_DESCR_PRIM(struct wifi_settings_struct, password, JSON_TOK_STRING),
                };
                int ret = json_obj_parse(buffer, len, descr, ARRAY_SIZE(descr), &wifi_settings);
                if (ret < 0) {
                    LOG_ERR("Failed to parse JSON data: %d", ret);
                    return snprintf(
                            buffer, len, "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
                }

                // Save SSID and Password to flash storage
                if (flash_storage_write(SSID, wifi_settings.ssid, strlen(wifi_settings.ssid)) < 0) {
                    LOG_ERR("Failed to save SSID to flash storage");
                    return snprintf(buffer,
                            len,
                            "{\"status\":\"error\", \"message\":\"Failed to save SSID\"}");
                }
                if (flash_storage_write(
                            PASSWORD, wifi_settings.password, strlen(wifi_settings.password))
                        < 0) {
                    LOG_ERR("Failed to save Password to flash storage");
                    return snprintf(buffer,
                            len,
                            "{\"status\":\"error\", \"message\":\"Failed to save Password\"}");
                }
            }

            return snprintf(buffer, len, "{\"status\":\"success\"}");
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int update_ros_settings_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (len > 0) {
                struct ros_settings_struct {
                    char *namespace;
                    char *agent_ip;
                };

                struct ros_settings_struct ros_settings = { 0 };

                // Parse JSON data and update network settings
                struct json_obj_descr descr[] = {
                    JSON_OBJ_DESCR_PRIM(struct ros_settings_struct, namespace, JSON_TOK_STRING),
                    JSON_OBJ_DESCR_PRIM(struct ros_settings_struct, agent_ip, JSON_TOK_STRING),
                };
                int ret = json_obj_parse(buffer, len, descr, ARRAY_SIZE(descr), &ros_settings);
                if (ret < 0) {
                    LOG_ERR("Failed to parse JSON data: %d", ret);
                    return snprintf(
                            buffer, len, "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
                }

                // Save Namespace and Agent IP to flash storage
                if (flash_storage_write(
                            NAMESPACE, ros_settings.namespace, strlen(ros_settings.namespace))
                        < 0) {
                    LOG_ERR("Failed to save namespace to flash storage");
                    return snprintf(buffer,
                            len,
                            "{\"status\":\"error\", \"message\":\"Failed to save SSID\"}");
                }
                if (flash_storage_write(
                            AGENT_IP, ros_settings.agent_ip, strlen(ros_settings.agent_ip))
                        < 0) {
                    LOG_ERR("Failed to save agent IP to flash storage");
                    return snprintf(buffer,
                            len,
                            "{\"status\":\"error\", \"message\":\"Failed to save Password\"}");
                }
            }

            return snprintf(buffer, len, "{\"status\":\"success\"}");
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int ip_address_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;
    LOG_DBG("IP Address handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            struct net_if *iface = net_if_get_wifi_sta();
            if (!iface) {
                LOG_ERR("No default network interface found");
                return snprintf(buffer,
                        len,
                        "{\"status\":\"error\", \"message\":\"No network interface\"}");
            }

            struct in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
            if (!addr) {
                LOG_ERR("Failed to get global IPv4 address");
                return snprintf(
                        buffer, len, "{\"status\":\"error\", \"message\":\"No IP address\"}");
            }

            char data[256];
            snprintf(data,
                    sizeof(data),
                    "{\"ip_address\":\"%s\"}",
                    net_addr_ntop(AF_INET, addr, ip_address_buf, sizeof(ip_address_buf)));
            return snprintf(buffer, sizeof(data), "%s", data);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int namespace_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            char namespace[64];
            flash_storage_read(NAMESPACE, namespace, sizeof(namespace));

            return snprintf(buffer, sizeof(namespace_buf), "{\"namespace\":\"%s\"}", namespace);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int domain_id_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;
    LOG_DBG("Domain ID handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            return snprintf(buffer,
                    sizeof(domain_id_buf),
                    "{\"domain_id\":\"%d\"}",
                    CONFIG_ROS_ROS_DOMAIN_ID);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int micro_ros_status_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            enum states current_status = get_micro_ros_node_status();
            const char *status_str = get_micro_ros_node_status_string(current_status);

            return snprintf(
                    buffer, sizeof(micro_ros_status_buf), "{\"status\":\"%s\"}", status_str);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int reset_to_factory_settings(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            // Reset namespace to default
            flash_storage_write(NAMESPACE, CONFIG_ROS_NAMESPACE, strlen(CONFIG_ROS_NAMESPACE));
            flash_storage_write(
                    AGENT_IP, CONFIG_MICROROS_AGENT_IP, strlen(CONFIG_MICROROS_AGENT_IP));

            LOG_INF("Factory settings reset complete");
            return snprintf(buffer,
                    sizeof(micro_ros_status_buf),
                    "{\"status\":\"success\", \"message\":\"Factory settings reset\"}");
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int estop_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    printk("E-Stop handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (len > 0) {
                LOG_DBG("Received data: %.*s", len, buffer);

                struct estop_struct {
                    bool active;
                };

                struct estop_struct estop_data = { 0 };

                struct json_obj_descr descr[] = {
                    JSON_OBJ_DESCR_PRIM(struct estop_struct, active, JSON_TOK_TRUE),
                };
                int ret = json_obj_parse(buffer, len, descr, ARRAY_SIZE(descr), &estop_data);
                if (ret < 0) {
                    LOG_ERR("Failed to parse JSON data: %d", ret);
                    return snprintf(
                            buffer, len, "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
                }

                if (estop_data.active) {
                    LOG_INF("E-Stop activated");
                    disable_cmd_vel();
                } else {
                    LOG_INF("E-Stop released");
                    enable_cmd_vel();
                }
            }

            return snprintf(buffer, len, "{\"status\":\"success\"}");
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int restart_system_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            LOG_INF("System restart initiated");
            sys_reboot(SYS_REBOOT_COLD);

            return snprintf(buffer,
                    sizeof(micro_ros_status_buf),
                    "{\"status\":\"success\", \"message\":\"System restart initiated\"}");
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static int agent_ip_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;
    LOG_DBG("Agent IP handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                response_sent = false;
                return 0;
            }

            response_sent = true;

            char agent_ip[64];
            flash_storage_read(AGENT_IP, agent_ip, sizeof(agent_ip));

            printk("Agent IP: %s", agent_ip);

            return snprintf(buffer, sizeof(agent_ip_buf), "{\"agent_ip\":\"%s\"}", agent_ip);
        }
        default: {
            LOG_WRN("Unexpected status %d", status);
            return -1;
        }
    }
}

static struct http_resource_detail_dynamic uptime_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = uptime_handler,
    .data_buffer = uptime_buf,
    .data_buffer_len = sizeof(uptime_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic version_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = version_handler,
    .data_buffer = version_buf,
    .data_buffer_len = sizeof(version_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic ssid_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = ssid_handler,
    .data_buffer = ssid_buf,
    .data_buffer_len = sizeof(ssid_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic update_network_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_POST),
		},
	.cb = update_network_handler,
    .data_buffer = update_network_buf,
    .data_buffer_len = sizeof(update_network_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic update_ros_settings_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_POST),
		},
	.cb = update_ros_settings_handler,
    .data_buffer = update_ros_settings_buf,
    .data_buffer_len = sizeof(update_ros_settings_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic ip_address_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = ip_address_handler,
    .data_buffer = ip_address_buf,
    .data_buffer_len = sizeof(ip_address_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic namespace_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = namespace_handler,
    .data_buffer = namespace_buf,
    .data_buffer_len = sizeof(namespace_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic domain_id_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = domain_id_handler,
    .data_buffer = domain_id_buf,
    .data_buffer_len = sizeof(domain_id_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic micro_ros_status_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = micro_ros_status_handler,
    .data_buffer = micro_ros_status_buf,
    .data_buffer_len = sizeof(micro_ros_status_buf),
	.user_data = NULL,
};

static struct http_resource_detail_dynamic estop_resource_detail = {
    .common = {
        .type = HTTP_RESOURCE_TYPE_DYNAMIC,
        .bitmask_of_supported_http_methods = BIT(HTTP_POST),
    },
    .cb = estop_handler,
    .data_buffer = estop_buf,
    .data_buffer_len = sizeof(estop_buf),
    .user_data = NULL,
};

static struct http_resource_detail_dynamic factory_reset_resource_detail = {
    .common = {
        .type = HTTP_RESOURCE_TYPE_DYNAMIC,
        .bitmask_of_supported_http_methods = BIT(HTTP_POST),
    },
    .cb = reset_to_factory_settings,
    .data_buffer = factory_reset_buf,
    .data_buffer_len = sizeof(factory_reset_buf),
    .user_data = NULL,
};

static struct http_resource_detail_dynamic restart_system_resource_detail = {
    .common = {
        .type = HTTP_RESOURCE_TYPE_DYNAMIC,
        .bitmask_of_supported_http_methods = BIT(HTTP_POST),
    },
    .cb = restart_system_handler,
    .data_buffer = factory_reset_buf,
    .data_buffer_len = sizeof(factory_reset_buf),
    .user_data = NULL,
};

static struct http_resource_detail_dynamic agent_ip_resource_detail = {
    .common = {
        .type = HTTP_RESOURCE_TYPE_DYNAMIC,
        .bitmask_of_supported_http_methods = BIT(HTTP_GET),
    },
    .cb = agent_ip_handler,
    .data_buffer = agent_ip_buf,
    .data_buffer_len = sizeof(agent_ip_buf),
    .user_data = NULL,
};

static uint16_t web_interface_service_port = 80;
HTTP_SERVICE_DEFINE(web_interface_service, NULL, &web_interface_service_port, 1, 10, NULL);

HTTP_RESOURCE_DEFINE(
        index_html_gz_resource, web_interface_service, "/", &index_html_gz_resource_detail);

HTTP_RESOURCE_DEFINE(
        main_js_gz_resource, web_interface_service, "/main.js", &main_js_gz_resource_detail);

HTTP_RESOURCE_DEFINE(
        style_css_gz_resource, web_interface_service, "/style.css", &style_css_gz_resource_detail);

HTTP_RESOURCE_DEFINE(uptime_resource, web_interface_service, "/uptime", &uptime_resource_detail);

HTTP_RESOURCE_DEFINE(version_resource, web_interface_service, "/version", &version_resource_detail);

HTTP_RESOURCE_DEFINE(ssid_resource, web_interface_service, "/ssid", &ssid_resource_detail);

HTTP_RESOURCE_DEFINE(update_network_resource,
        web_interface_service,
        "/update_network",
        &update_network_resource_detail);

HTTP_RESOURCE_DEFINE(update_ros_settings_resource,
        web_interface_service,
        "/update_ros_settings",
        &update_ros_settings_resource_detail);

HTTP_RESOURCE_DEFINE(
        ip_address_resource, web_interface_service, "/ip", &ip_address_resource_detail);

HTTP_RESOURCE_DEFINE(
        namespace_resource, web_interface_service, "/namespace", &namespace_resource_detail);

HTTP_RESOURCE_DEFINE(
        domain_id_resource, web_interface_service, "/domain_id", &domain_id_resource_detail);

HTTP_RESOURCE_DEFINE(micro_ros_status_resource,
        web_interface_service,
        "/micro_ros_status",
        &micro_ros_status_resource_detail);

HTTP_RESOURCE_DEFINE(estop_resource, web_interface_service, "/estop", &estop_resource_detail);

HTTP_RESOURCE_DEFINE(factory_reset_resource,
        web_interface_service,
        "/factory_reset",
        &factory_reset_resource_detail);

HTTP_RESOURCE_DEFINE(restart_system_resource,
        web_interface_service,
        "/restart_system",
        &restart_system_resource_detail);

HTTP_RESOURCE_DEFINE(
        agent_ip_resource, web_interface_service, "/agent_ip", &agent_ip_resource_detail);

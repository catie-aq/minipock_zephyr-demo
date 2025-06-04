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
#include <zephyr/sys/util_macro.h>

#include "flash_storage.h"
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
static uint8_t ip_address_buf[256];
static uint8_t namespace_buf[256];

static int uptime_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;
    // LOG_DBG("Uptime handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            /* A payload is not expected with the GET request. Ignore any data and wait until
             * final callback before sending response
             */
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                /* Response already sent, return 0 to indicate to server that the callback
                 * does not need to be called again.
                 */
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
    LOG_DBG("Version handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            response_sent = false;
            return 0;
        }

        case HTTP_SERVER_DATA_MORE: {
            /* A payload is not expected with the GET request. Ignore any data and wait until
             * final callback before sending response
             */
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (response_sent) {
                /* Response already sent, return 0 to indicate to server that the callback
                 * does not need to be called again.
                 */
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
    LOG_DBG("SSID handler status %d", status);

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
    printk("Update Network handler status %d", status);

    switch (status) {
        case HTTP_SERVER_DATA_ABORTED: {
            return 0;
        }

        case HTTP_SERVER_DATA_FINAL: {
            if (len > 0) {
                LOG_DBG("Received data: %.*s", len, buffer);

                // char encoded[] = "{\"ssid\":\"sd\",\"password\":\"sd\"";

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
    LOG_DBG("Namespace handler status %d", status);

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

HTTP_RESOURCE_DEFINE(
        ip_address_resource, web_interface_service, "/ip", &ip_address_resource_detail);

HTTP_RESOURCE_DEFINE(
        namespace_resource, web_interface_service, "/namespace", &namespace_resource_detail);

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
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util_macro.h>

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

static int uptime_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    static bool response_sent;
    LOG_DBG("Uptime handler status %d", status);

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

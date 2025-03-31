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

static int uptime_handler(struct http_client_ctx *client,
        enum http_data_status status,
        uint8_t *buffer,
        size_t len,
        void *user_data)
{
    int ret;

    LOG_DBG("Uptime handler status %d", status);

    /* A payload is not expected with the GET request. Ignore any data and wait until
     * final callback before sending response
     */
    if (status == HTTP_SERVER_DATA_FINAL) {
        ret = snprintf(uptime_buf, sizeof(uptime_buf), "%" PRId64, k_uptime_get());
        if (ret < 0) {
            LOG_ERR("Failed to snprintf uptime, err %d", ret);
            return ret;
        }
    }

    return ret;
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

static uint16_t web_interface_service_port = 80;
HTTP_SERVICE_DEFINE(web_interface_service, NULL, &web_interface_service_port, 1, 10, NULL);

HTTP_RESOURCE_DEFINE(
        index_html_gz_resource, web_interface_service, "/", &index_html_gz_resource_detail);

HTTP_RESOURCE_DEFINE(
        main_js_gz_resource, web_interface_service, "/main.js", &main_js_gz_resource_detail);

HTTP_RESOURCE_DEFINE(
        style_css_gz_resource, web_interface_service, "/style.css", &style_css_gz_resource_detail);

HTTP_RESOURCE_DEFINE(uptime_resource, web_interface_service, "/uptime", &uptime_resource_detail);

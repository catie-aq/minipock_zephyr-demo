/*
 * Copyright (c) 2025, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(flash_storage, LOG_LEVEL_DBG);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

static struct nvs_fs fs;

int flash_storage_init(void)
{
    int rc;
    struct flash_pages_info info;

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        printk("Flash device %s is not ready\n", fs.flash_device->name);
        return 0;
    }
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        printk("Unable to get page info\n");
        return 0;
    }
    fs.sector_size = info.size;
    fs.sector_count = 3U;

    rc = nvs_mount(&fs);
    if (rc) {
        printk("Flash Init failed\n");
        return 0;
    }

    return 0;
}

int flash_storage_write(const char *key, const char *value)
{
    int rc;
    rc = nvs_write(&fs, (uint16_t)key, value, strlen(value) + 1);
    if (rc < 0) {
        LOG_ERR("Flash Write failed");
        return rc;
    }
    return 0;
}

int flash_storage_read(const char *key, char *value, size_t max_len)
{
    int rc;
    rc = nvs_read(&fs, (uint16_t)key, value, max_len);
    if (rc < 0) {
        LOG_ERR("Flash Read failed");
        return rc;
    }
    return 0;
}

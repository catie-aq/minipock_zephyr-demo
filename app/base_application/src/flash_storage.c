/*
 * Copyright (c) 2025, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(flash_storage, LOG_LEVEL_DBG);

#define MAX_KEY_LEN 64

#define FLASH_PARTITION storage_partition
#define FLASH_PARTITION_DEVICE FIXED_PARTITION_DEVICE(FLASH_PARTITION)
#define FLASH_PARTITION_OFFSET FIXED_PARTITION_OFFSET(FLASH_PARTITION)

static const struct device *flash_dev;
static uint32_t sector_size;

int flash_storage_init(void)
{
    struct flash_pages_info info;

    flash_dev = FLASH_PARTITION_DEVICE;
    if (!device_is_ready(flash_dev)) {
        printk("Flash device %s is not ready\n", flash_dev->name);
        return -ENODEV;
    }

    int rc = flash_get_page_info_by_offs(flash_dev, FLASH_PARTITION_OFFSET, &info);
    if (rc) {
        printk("Unable to get page info\n");
        return rc;
    }

    sector_size = info.size;

    return 0;
}

int flash_storage_write(int key, const void *data, size_t len)
{
    int rc;

    uint8_t buf[512];
    rc = flash_read(flash_dev, FLASH_PARTITION_OFFSET, buf, sizeof(buf));
    if (rc) {
        printk("Flash Read failed %d, key: %d", rc, key);
        return rc;
    }
    memcpy(&buf[key * MAX_KEY_LEN], data, len);
    buf[key * MAX_KEY_LEN + len] = '\0';

    rc = flash_erase(flash_dev, FLASH_PARTITION_OFFSET, sector_size);
    if (rc) {
        printk("Flash Erase failed %d", rc);
        return rc;
    }

    rc = flash_write(flash_dev, FLASH_PARTITION_OFFSET, buf, sizeof(buf));
    if (rc) {
        printk("Flash Write failed %d", rc);
        return rc;
    }
    return 0;
}

int flash_storage_read(int key, void *data, size_t len)
{
    int rc;

    if (len > MAX_KEY_LEN) {
        LOG_ERR("Data length exceeds maximum key length");
        return -EINVAL;
    }

    rc = flash_read(flash_dev, FLASH_PARTITION_OFFSET + key * MAX_KEY_LEN, data, len);
    if (rc) {
        LOG_ERR("Flash Read failed %d, key: %d", rc, key);
        return rc;
    }
    return 0;
}

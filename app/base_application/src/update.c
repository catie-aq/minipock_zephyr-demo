#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "update.h"

LOG_MODULE_REGISTER(update, LOG_LEVEL_DBG);

void update_write_chunk(const uint16_t id, const uint8_t *chunk, size_t size)
{
    LOG_DBG("Writing chunk %d", id);

    const struct flash_area *flash_area;
    int rc = flash_area_open(SECOND_SLOT_PARTITION_ID, &flash_area);
    if (rc) {
        LOG_ERR("Cannot open flash area");
        return;
    }

    if (id == 0) {
        rc = flash_area_erase(flash_area, 0, flash_area->fa_size);
        if (rc) {
            LOG_ERR("Cannot erase flash area");
            flash_area_close(flash_area);
            return;
        }
    }

    size_t chunk_size = size;
    if (size % 32) {
        chunk_size = size + 32 - (size % 32);
    }

    rc = flash_area_write(flash_area, id * UPDATE_CHUNK_SIZE, chunk, chunk_size);
    if (rc) {
        LOG_ERR("Cannot write to flash area");
        flash_area_close(flash_area);
        return;
    }

    flash_area_close(flash_area);
}

void update_get_current_version(uint8_t *major, uint8_t *minor, uint8_t *revision)
{
    struct mcuboot_img_header header;
    boot_read_bank_header(PRIMARY_SLOT_PARTITION_ID, &header, sizeof(header));

    *major = header.h.v1.sem_ver.major;
    *minor = header.h.v1.sem_ver.minor;
    *revision = header.h.v1.sem_ver.revision;
}

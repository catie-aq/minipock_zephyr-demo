#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "update.h"

LOG_MODULE_REGISTER(update, LOG_LEVEL_DBG);

static struct update_interface *update_interface;

static struct update_params update_params;

static enum states {
    WAITING_UPDATE,
    UPDATE_AVAILABLE,
    UPDATE_IN_PROGRESS,
    UPDATE_COMPLETED,
} state;

struct update_status {
    enum states current_state;
    int progress_percentage;
    int last_chunk_id;
    bool in_progress;
};

static struct update_status update_status = {
    .in_progress = false,
    .progress_percentage = 0,
    .last_chunk_id = -1,
};

void update_request_params(void)
{
    LOG_DBG("Requesting update params");
    if (update_interface->request_update_params != NULL) {
        update_interface->request_update_params();
    }
}

void update_receive_params(const struct update_params *params)
{
    LOG_DBG("Received update params: %s, %d", params->version, params->size);
    update_params = *params;

    update_status.current_state = UPDATE_AVAILABLE;
}

void update_start(void)
{
    LOG_DBG("Starting update with version: %s, size: %d",
            update_params.version,
            update_params.size);
    update_status.current_state = UPDATE_IN_PROGRESS;

    if (update_interface->request_update != NULL) {
        update_interface->request_update();
    }
}

void update_write_chunk(const uint8_t id, const uint8_t *chunk, size_t size)
{
    LOG_INF("Writing chunk %d", id);
    // TODO: Store the chunk in flash memory

    update_status.last_chunk_id = id;
    update_status.progress_percentage = (id * size * 100) / update_params.size;

    int percentage = (id * size * 100) / update_params.size;
    LOG_INF("Update progress: %d%%", percentage);
}

void update_init(struct update_interface *trigger)
{
    LOG_DBG("Initializing update");
    update_interface = trigger;
}

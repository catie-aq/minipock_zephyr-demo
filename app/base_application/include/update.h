#ifndef UPDATE_H
#define UPDATE_H

#include <zephyr/storage/flash_map.h>

#define UPDATE_CHUNK_SIZE 1024

#define PRIMARY_SLOT_PARTITION_ID FIXED_PARTITION_ID(slot0_partition)
#define SECOND_SLOT_PARTITION_ID FIXED_PARTITION_ID(slot1_partition)

struct update_interface {
    void (*request_update_params)(void);
    void (*request_update)(void);
};

struct update_params {
    const char *version;
    size_t size;
};

#endif // UPDATE_H

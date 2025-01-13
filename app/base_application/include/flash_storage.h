#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

int flash_storage_init(void);
int flash_storage_write(const char *key, const char *value);
int flash_storage_read(const char *key, char *value, size_t max_len);

#endif // FLASH_STORAGE_H

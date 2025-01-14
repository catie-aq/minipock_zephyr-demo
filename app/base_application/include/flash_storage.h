#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#define NAMESPACE 0
#define SSID 1
#define PASSWORD 2
#define CHANNEL 3
#define AGENT_IP 4

int flash_storage_init(void);
int flash_storage_write(const int key, const char *value, size_t len);
int flash_storage_read(const int key, char *value, size_t max_len);

#endif // FLASH_STORAGE_H

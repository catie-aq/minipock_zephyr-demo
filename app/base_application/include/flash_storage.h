#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#define NAMESPACE 0
#define SSID 1
#define PASSWORD 2
#define CHANNEL 3
#define AGENT_IP 4

#define SD_MODE 0 // 0: Use SD card
#define STORAGE_MODE 1 // 1: Use Flash storage

#define HOTSPOT_SSID "ShrekTelecomP" // Default SSID Used if not in using SD_MODE
#define HOTSPOT_PSK "Shrekos36" // Default Password Used if not in using SD_MODE
#define MICRO_ROS_AGENT_IP "192.168.169.25" // Default IP Used if not in using SD_MODE

int flash_storage_init(void);
int flash_storage_write(const int key, const char *value, size_t len);
int flash_storage_read(const int key, char *value, size_t max_len);

#endif // FLASH_STORAGE_H

#include <zephyr/shell/shell.h>

#include "flash_storage.h"

static int cmd_flash_storage_read(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(shell, "Usage: %s <address>", argv[0]);
        return -1;
    }

    uint32_t address = strtoul(argv[1], NULL, 0);
    uint32_t data;
    flash_storage_read(address, &data, sizeof(data));
    shell_print(shell, "Read data: 0x%08x", data);

    return 0;
}

static int cmd_flash_storage_write(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_print(shell, "Usage: %s <address> <data>", argv[0]);
        return -1;
    }

    uint32_t address = strtoul(argv[1], NULL, 0);
    uint32_t data = strtoul(argv[2], NULL, 0);
    flash_storage_write(address, data);
    shell_print(shell, "Write data: 0x%08x", data);

    return 0;
}

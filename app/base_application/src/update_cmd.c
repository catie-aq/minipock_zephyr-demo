#include <zephyr/shell/shell.h>

#include "micro_ros_node.h"
#include "update.h"

static int cmd_update(const struct shell *shell, size_t argc, char **argv)
{
    micro_ros_node_get_last_version();
    return 0;
}

static int cmd_current_version(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t major, minor, revision;
    update_get_current_version(&major, &minor, &revision);
    shell_print(shell, "v%d.%d.%d", major, minor, revision);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_micro_ros,
        SHELL_CMD(update, NULL, "Update", cmd_update),
        SHELL_CMD(version, NULL, "Current version", cmd_current_version),
        SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(firmware, &sub_micro_ros, "Firmware commands", NULL);

#include <zephyr/shell/shell.h>

#include "micro_ros_node.h"

static int cmd_update(const struct shell *shell, size_t argc, char **argv)
{
    micro_ros_node_get_last_version();
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
        sub_micro_ros, SHELL_CMD(update, NULL, "Update", cmd_update), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(micro_ros, &sub_micro_ros, "Micro-ROS commands", NULL);

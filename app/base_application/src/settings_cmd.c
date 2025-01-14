#include <zephyr/shell/shell.h>

#include "flash_storage.h"

static int cmd_set_wifi_ssid(const struct shell *shell, size_t argc, char **argv)
{
    int ret = -1;

    if (argc > 2) {
        shell_print(shell, "Usage: %s [ssid]", argv[0]);
        return ret;
    } else if (argc == 1) {
        char ssid[32];
        ret = flash_storage_read(SSID, ssid, sizeof(ssid));
        shell_print(shell, "SSID: %s", ssid);
        return 0;
    } else {
        ret = flash_storage_write(SSID, argv[1], strlen(argv[1]));
        shell_print(shell, "Set SSID: %s", argv[1]);
    }

    return ret;
}

static int cmd_set_wifi_password(const struct shell *shell, size_t argc, char **argv)
{
    int ret = -1;

    if (argc > 2) {
        shell_print(shell, "Usage: %s [password]", argv[0]);
        return ret;
    } else if (argc == 1) {
        char password[32];
        ret = flash_storage_read(PASSWORD, password, sizeof(password));
        shell_print(shell, "Password: %s", password);
        return 0;
    } else {
        ret = flash_storage_write(PASSWORD, argv[1], strlen(argv[1]));
        shell_print(shell, "Set password: %s", argv[1]);
    }

    return ret;
}

static int cmd_set_wifi_channel(const struct shell *shell, size_t argc, char **argv)
{
    int ret = -1;

    if (argc > 2) {
        shell_print(shell, "Usage: %s [channel]", argv[0]);
        return ret;
    } else if (argc == 1) {
        uint8_t channel;
        ret = flash_storage_read(CHANNEL, &channel, sizeof(channel));
        shell_print(shell, "Channel: %d", channel);
        return 0;
    } else {
        uint8_t channel = (uint8_t)strtol(argv[1], NULL, 10);
        ret = flash_storage_write(CHANNEL, &channel, sizeof(channel));
        shell_print(shell, "Set channel: %d", channel);
    }

    return ret;
}

static int cmd_set_namespace(const struct shell *shell, size_t argc, char **argv)
{
    int ret = -1;

    if (argc > 2) {
        shell_print(shell, "Usage: %s [namespace]", argv[0]);
        return ret;
    } else if (argc == 1) {
        char namespace[32];
        ret = flash_storage_read(NAMESPACE, namespace, sizeof(namespace));
        shell_print(shell, "Namespace: %s", namespace);
        return 0;
    } else {
        ret = flash_storage_write(NAMESPACE, argv[1], strlen(argv[1]));
        shell_print(shell, "Set namespace: %s", argv[1]);
    }

    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(settings,
        SHELL_CMD(ssid, NULL, "Set SSID", cmd_set_wifi_ssid),
        SHELL_CMD(password, NULL, "Set password", cmd_set_wifi_password),
        SHELL_CMD(channel, NULL, "Set channel", cmd_set_wifi_channel),
        SHELL_CMD(namespace, NULL, "Set namespace", cmd_set_namespace),
        SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(settings, &settings, "Settings commands", NULL);

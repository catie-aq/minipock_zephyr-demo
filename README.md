![banner](imgs/minipock_banner_dark_theme.svg#gh-dark-mode-only)
![banner](imgs/minipock_banner_light_theme.svg#gh-light-mode-only)

[![Build](https://github.com/catie-aq/minipock_zephyr-demo/actions/workflows/build.yaml/badge.svg)](https://github.com/catie-aq/minipock_zephyr-demo/actions/workflows/build.yaml)

# MiniPock Firmware

This repository contains the firmware based on Zephyr OS for the MiniPock.

## Usage

> [!NOTE]
> You can use Dev Containers to develop in a consistent environment. This ensures that all dependencies and tools are correctly set up.

### Initialize with Dev Container

- Clone the repository

```
mkdir minipock_workspace
cd minipock_workspace
git clone https://github.com/catie-aq/zephyr_minipock minipock
```

- Open VSCode and open the Dev Container
- Initialize the workspace
```bash
west init -l minipock
west update
```

### Initialize without Dev Container

- Install the dependencies

```bash
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```

- Initialize the workspace

```bash
mkdir minipock_workspace
cd minipock_workspace
west init -m https://github.com/catie-aq/zephyr_minipock minipock
west update
```

### Build and Flash

- Compile the project

```bash
west build -b zest_core_stm32h753zi app/<project>
```

- Program the target device

```bash
west flash
```

### Generate key file for MCUBoot

```bash
imgtool keygen -k root-rsa-2048.pem -t rsa-2048
```

### Serial micro-ROS Agent

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## Examples
- [Base Application](app/base_application): Simple application to control the MiniPock.

![banner](imgs/MiniPock_banner_dark_theme.png#gh-dark-mode-only)
![banner](imgs/MiniPock_banner_light_theme.png#gh-light-mode-only)

# MiniPock Firmware

This repository contains the firmware based on Zephyr OS for the MiniPock.

## Requirements

```bash
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```

## Usage

- Initialize Zephyr workspace
```bash
west init -m https://github.com/catie-aq/zephyr_minipock minipock
cd minipock
west update
```

- Compile the project
```bash
west build -b zest_core_stm32l4a6rg app/<project> -- -DBOARD_ROOT=path/to/board
```

- Program the target device
```bash
west flash
```

## Serial micro-ROS Agent

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB[X] -v6
```

## Examples
- [Base Application](app/base_application): Simple application to control the MiniPock.
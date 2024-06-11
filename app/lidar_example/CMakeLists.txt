# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/micro_ros_zephyr_module/modules/libmicroros/)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lidar_example)

target_sources(app PRIVATE ${proto_sources} src/main.c)

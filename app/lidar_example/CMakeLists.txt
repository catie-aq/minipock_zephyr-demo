# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/micro_ros_zephyr_module/modules/libmicroros/)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lidar_example)

# Apply Git patch before build
execute_process(
    COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/../fix_static_init_gnu.patch
    WORKING_DIRECTORY ${ZEPHYR_BASE}
    RESULT_VARIABLE result
)

execute_process(
    COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/../fix_zest_core_stm32h753zi_ethernet.patch
    WORKING_DIRECTORY ${ZEPHYR_BASE}/../6tron/boards/zest_core_stm32h753zi
    RESULT_VARIABLE result
)

target_sources(app PRIVATE ${proto_sources} src/main.c)

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/micro_ros_zephyr_module/modules/libmicroros/)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base_application)

# Apply Git patch before build
execute_process(
    COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/../fix_static_init_gnu.patch
    WORKING_DIRECTORY ${ZEPHYR_BASE}
    RESULT_VARIABLE result
)

list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

zephyr_nanopb_sources(app src/minipock.proto)

add_subdirectory(drivers)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE ${proto_sources})
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/scan.c)
target_sources(app PRIVATE src/base_interface.c)
target_sources(app PRIVATE src/micro_ros_node.c)
target_sources(app PRIVATE src/update.c)

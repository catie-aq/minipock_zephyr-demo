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

option(INCLUDE_HTML_CONTENT "Include the HTML content" ON)

add_subdirectory(drivers)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE ${proto_sources})
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/scan.c)
target_sources(app PRIVATE src/base_interface.c)
target_sources(app PRIVATE src/micro_ros_node.c)
target_sources(app PRIVATE src/update.c)
target_sources(app PRIVATE src/update_cmd.c)
target_sources(app PRIVATE src/flash_storage.c)
target_sources(app PRIVATE src/settings_cmd.c)
target_sources(app PRIVATE src/web_interface.c)

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)

target_link_libraries(app PRIVATE zephyr_interface zephyr)

zephyr_linker_sources(SECTIONS sections-rom.ld)
zephyr_linker_section_ifdef(CONFIG_NET_SAMPLE_HTTP_SERVICE NAME
				http_resource_desc_web_interface_service
				KVMA RAM_REGION GROUP RODATA_REGION
				SUBALIGN ${CONFIG_LINKER_ITERABLE_SUBALIGN})

foreach(web_resource
  index.html
  style.css
  main.js
    )
  generate_inc_file_for_target(
    app
    src/static_web_resources/${web_resource}
    ${gen_dir}/${web_resource}.gz.inc
    --gzip
  )
endforeach()

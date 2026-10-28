#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB_RECURSE app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE ${app_sources})
# NORDIC SDK APP END

target_include_directories(app PRIVATE src)

# riftcv1: the camera beacon, from riftcv1-mocap's firmware/rift_common (a west
# project beside nrf/), sharing the chip with ESB (RIFT_SHARED)
if(NOT DEFINED RIFT_COMMON)
  set(RIFT_COMMON ${ZEPHYR_BASE}/../riftcv1-mocap/firmware/rift_common)
endif()
target_include_directories(app PRIVATE ${RIFT_COMMON})
target_sources(app PRIVATE
  ${RIFT_COMMON}/beacon_tx.c
  ${RIFT_COMMON}/beacon_app.c
  ${RIFT_COMMON}/led_pulse.c)
target_compile_definitions(app PRIVATE RIFT_SHARED)

# riftcv1: the acknowledgement timestamps need a hook in ESB's PRX interrupt,
# so NCS's subsys/esb/esb.c is swapped for the patched copy in esb/
set(ESB_LIB ..__nrf__subsys__esb)
get_target_property(esb_sources ${ESB_LIB} SOURCES)
list(FILTER esb_sources EXCLUDE REGEX "(^|/)esb\\.c$")
set_property(TARGET ${ESB_LIB} PROPERTY SOURCES ${esb_sources} ${CMAKE_CURRENT_SOURCE_DIR}/esb/esb.c)
target_include_directories(${ESB_LIB} PRIVATE ${ZEPHYR_NRF_MODULE_DIR}/subsys/esb ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Force recompile for updated build timestamp
add_custom_target(touch_util_h ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/src/build_defines.h
)

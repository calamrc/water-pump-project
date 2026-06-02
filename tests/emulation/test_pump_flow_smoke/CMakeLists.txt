# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_pump_flow_smoke)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include/zbus
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/drivers/sensor/yf_s201c/src
)

# Satisfy custom drivers that reference APP-level logging symbols
# (defined in the main app but not present in this minimal test)
target_compile_definitions(app PRIVATE
  CONFIG_APP_LOG_LEVEL=3
)

target_sources(app PRIVATE
  src/test_pump_flow_smoke.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/pump_state_machine.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/flow_processor.c
)

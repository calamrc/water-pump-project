# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_pump_state_machine)

# Include the app headers so we can use pump_state_machine.h
zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include
)

# Pull in the pure state machine implementation + the test
target_sources(app PRIVATE
  src/test_pump_state_machine.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/pump_state_machine.c
)

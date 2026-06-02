# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_timer_state_machine)

zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include/timer
)

target_sources(app PRIVATE
  src/test_timer_state_machine.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/timer/timer_state_machine.c
)

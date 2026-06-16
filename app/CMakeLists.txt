# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

# Include directories for UI modules
target_include_directories(app PRIVATE
  include
  include/display
  include/input
  include/timer
  include/zbus
)

target_sources(app PRIVATE
  src/main.c
  src/flow_analyzer.c
  src/flow_analyzer_thread.c
  src/error_handler.c
  src/thread_manager.c
  src/ui_manager.c
  src/display/display_manager.c
  src/input/input_manager.c
  src/timer/timer_state_machine.c
  src/zbus/channels.c
)

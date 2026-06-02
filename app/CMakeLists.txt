# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

# Include directories
target_include_directories(app PRIVATE
  include
  include/display
  include/input
  include/timer
  include/services
  include/zbus
  include/pump
  src
  src/zbus
)

target_sources(app PRIVATE
  src/main.c
  src/flow_analyzer.c
  src/error_handler.c
  src/display/display_manager.c
  src/input/input_manager.c
  src/timer/timer_state_machine.c
  # Old flow_service.c fully removed during legacy cleanup.
  # New zbus-centric services (Phase 1 skeletons + channels)
  src/zbus/channels.c
  src/services/flow_sensor_service.c
  src/services/pump_service.c
  src/services/timer_service.c
  src/services/ui_service.c
  src/services/safety_service.c
  src/pump_state_machine.c
  src/flow_processor.c
  src/flow_analyzer_core.c
  src/pump/pump_demand.c
  src/pump/flow_stabilization_contract.c
)

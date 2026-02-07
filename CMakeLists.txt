# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(water_pump)

zephyr_syscall_include_directories(include)
zephyr_include_directories(include)
add_subdirectory(drivers)

# Application source files
target_sources(app PRIVATE
    app/src/main.c
    app/src/error_handler.c
    app/src/flow_analyzer.c
    app/src/safety_monitor.c
)

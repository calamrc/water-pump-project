# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

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

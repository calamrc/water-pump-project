# Copyright (c) 2025 Ramon Cristopher Calam
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(yf_s201c_driver_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(app PRIVATE
  src/yf_s201c_driver_test.c
)

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(libpeci-test)

set(common_path ${CMAKE_CURRENT_LIST_DIR})
FILE(GLOB app_sources ${common_path}/libpeci/peci.c)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE ${common_path}/libpeci)
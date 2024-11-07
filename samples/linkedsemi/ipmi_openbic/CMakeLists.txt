# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE src/ipmi/ipmi.c)
target_sources(app PRIVATE src/ipmi/app_handler.c)
target_sources(app PRIVATE src/ipmi/chassis_handler.c)
target_sources(app PRIVATE src/pal/ipmi/plat_ipmi.c)

target_sources(app PRIVATE src/lib/libutil.c)
target_sources(app PRIVATE src/lib/power_status.c)

target_include_directories(app PRIVATE src/)
target_include_directories(app PRIVATE src/ipmi/include)
target_include_directories(app PRIVATE src/ipmb)
target_include_directories(app PRIVATE src/lib)
target_include_directories(app PRIVATE src/pal/ipmi/include)
target_include_directories(app PRIVATE src/kcs)

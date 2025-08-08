cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wolfssl_tests)

FILE(GLOB app_sources echoserver.c main.c)
target_sources(app PRIVATE ${app_sources})

add_definitions(-DWOLFSSL_ZEPHYR)
add_definitions(-DWOLFSSL_USER_SETTINGS)

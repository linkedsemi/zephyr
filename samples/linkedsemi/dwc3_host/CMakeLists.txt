# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dwc3_host)

target_sources(app PRIVATE src/main.c src/xhci.c src/xhci_port.c)
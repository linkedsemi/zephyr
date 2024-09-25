# Copyright (c) 2024, Alibaba
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--iface=cJTAG" "--device=LSQSH" "--speed=8000" "--erase" "--tool-opt=-JTAGConf -1,-1" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

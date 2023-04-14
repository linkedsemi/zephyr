# SPDX-License-Identifier: Apache-2.0

set_property(TARGET runners_yaml_props_target PROPERTY hex_file "production.hex")

board_runner_args(jlink "--device=LE501X" "--speed=4000" "--erase" "--reset-after-load")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

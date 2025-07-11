#
# Copyright (c) 2017, NXP
#
# SPDX-License-Identifier: Apache-2.0
#

board_runner_args(jlink "--device=LPC54628J512" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
# Copyright (C) 2026 wentywenty
file(GLOB CAN_SRCS *.cpp)
add_library(motors_can STATIC ${CAN_SRCS})
target_include_directories(motors_can
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motors_can PUBLIC ${PUBLIC_DEPENDENCIES})
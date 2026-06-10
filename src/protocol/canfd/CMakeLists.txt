# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
file(GLOB CANFD_SRCS *.cpp)
add_library(motors_canfd STATIC ${CANFD_SRCS})
target_include_directories(motors_canfd
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motors_canfd PUBLIC ${PUBLIC_DEPENDENCIES})
# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
AUX_SOURCE_DIRECTORY(. SOURCE_LIST_XYN_MOTORS)
add_library(xyn_motors
        STATIC
        ${SOURCE_LIST_XYN_MOTORS}
)
target_include_directories(xyn_motors
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(xyn_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_canfd)

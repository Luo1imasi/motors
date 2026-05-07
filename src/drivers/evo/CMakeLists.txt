AUX_SOURCE_DIRECTORY(. SOURCE_LIST_EVO_MOTORS)
add_library(evo_motors
        STATIC
        ${SOURCE_LIST_EVO_MOTORS}
)
target_include_directories(evo_motors
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(evo_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_can motors_canfd)

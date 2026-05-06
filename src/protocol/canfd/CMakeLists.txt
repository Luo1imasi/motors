file(GLOB CANFD_SRCS *.cpp)
add_library(motors_canfd STATIC ${CANFD_SRCS})
target_include_directories(motors_canfd PUBLIC ./ ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(motors_canfd PUBLIC ${PUBLIC_DEPENDENCIES})
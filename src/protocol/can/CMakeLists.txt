file(GLOB CAN_SRCS *.cpp)
add_library(motors_can STATIC ${CAN_SRCS})
target_include_directories(motors_can PUBLIC ./ ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(motors_can PUBLIC ${PUBLIC_DEPENDENCIES})
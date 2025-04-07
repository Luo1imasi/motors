cmake_minimum_required(VERSION 3.8)
project(motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCES})
set(PUBLIC_DEPENDENCIES
    fmt::fmt spdlog::spdlog ${Boost_LIBRARIES} pthread)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/src)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PUBLIC_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs)

add_executable(${PROJECT_NAME}_node src/motors_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/src)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PUBLIC_DEPENDENCIES} ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC rclcpp sensor_msgs)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION bin)

ament_export_dependencies(sensor_msgs)
ament_package()

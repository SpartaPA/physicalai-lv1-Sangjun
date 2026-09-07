cmake_minimum_required(VERSION 3.8)
project(turtle_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(distance_publisher src/distance_publisher.cpp)
ament_target_dependencies(
  distance_publisher
  rclcpp
  std_msgs
  turtlesim
)

add_executable(distance_subscriber src/distance_subscriber.cpp)
ament_target_dependencies(
  distance_subscriber
  rclcpp
  std_msgs
)

install(
  TARGETS
    distance_publisher
    distance_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

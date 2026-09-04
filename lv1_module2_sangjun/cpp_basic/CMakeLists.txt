# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(robot_controller CXX)

set(CMAKE_CXX_STANDARD 17)             # C++17 사용
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot                    # 실행파일 robot을
    main.cpp                        # 이 소스들로 만든다
    motor.cpp
)
target_include_directories(robot PRIVATE include)   # 헤더 위치
# target_link_libraries(robot PRIVATE rclcpp::rclcpp)  # 외부 라이브러리 (ROS2에서)

add_executable(stop_distance
    stop_distance.cpp
)
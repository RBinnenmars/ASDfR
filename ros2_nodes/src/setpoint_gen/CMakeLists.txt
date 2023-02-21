cmake_minimum_required(VERSION 3.5)
project(setpoint_gen)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asdfr_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(sub_moving src/sub_moving_camera_output.cpp)
ament_target_dependencies(sub_moving 
  rclcpp 
  std_msgs
  OpenCV
  cv_bridge)

add_executable(setpoint_gen src/setpoint_generator.cpp)
ament_target_dependencies(setpoint_gen 
  rclcpp 
  sensor_msgs
  asdfr_interfaces)

install(TARGETS
  setpoint_gen
  sub_moving
  DESTINATION lib/${PROJECT_NAME})

ament_package()

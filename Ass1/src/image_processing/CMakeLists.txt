cmake_minimum_required(VERSION 3.5)
project(image_processing)

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
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(pub_brightness src/pub_brightness.cpp)
ament_target_dependencies(pub_brightness 
  rclcpp 
  sensor_msgs 
  std_msgs
  OpenCV
  cv_bridge)

add_executable(sub_brightness src/sub_brightness.cpp)
ament_target_dependencies(sub_brightness 
  rclcpp 
  std_msgs)

install(TARGETS
  pub_brightness
  sub_brightness
  DESTINATION lib/${PROJECT_NAME})

ament_package()

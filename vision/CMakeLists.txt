cmake_minimum_required(VERSION 3.8)
project(LAyR)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(visualizador_c src/visualizador_c.cpp)

ament_target_dependencies(
  visualizador_c
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS visualizador_c
  DESTINATION lib/${PROJECT_NAME})

ament_package()


cmake_minimum_required(VERSION 3.8)
project(cyberwaiter_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(ur_msgs REQUIRED)
find_package(cyberwaiter_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(kin_solver src/kin_control.cpp )
ament_target_dependencies(kin_solver
rclcpp std_msgs sensor_msgs 
tf2_geometry_msgs geometry_msgs 
kdl_parser orocos_kdl urdf 
rclcpp_action control_msgs trajectory_msgs 
cyberwaiter_msgs)

add_executable(gripper src/grip_chat.cpp )
ament_target_dependencies(gripper
rclcpp ur_msgs cyberwaiter_msgs) 

add_executable(navigator src/navigator.cpp )
ament_target_dependencies(navigator
rclcpp std_msgs geometry_msgs
orocos_kdl
cyberwaiter_msgs)

install(TARGETS
  kin_solver
  gripper
  navigator
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

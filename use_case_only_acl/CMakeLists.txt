cmake_minimum_required(VERSION 3.5)
project(use_case_only_acl)


# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
find_package(ros2_bdi_behaviours REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(bridge_interfaces REQUIRED)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(mclab_dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${dynamixel_workbench_INCLUDE_DIRS}
)

add_library(dynamixel_ros
  src/dynamixel_ros.cpp
  src/dynamixel_ctrl.cpp
)
# target_link_libraries(dynamixel_ros
#   ${dynamixel_sdk_LIBRARIES}
#   ${dynamixel_workbench_LIBRARIES}
# )
ament_target_dependencies(dynamixel_ros
  rclcpp
  std_msgs
  dynamixel_sdk
  dynamixel_workbench_toolbox
)

add_executable(dynamixel_node
  nodes/dynamixel_node.cpp)

ament_target_dependencies(dynamixel_node
  rclcpp
  std_msgs
  dynamixel_sdk
  dynamixel_workbench_toolbox
)
target_link_libraries(dynamixel_node
  dynamixel_ros
)

install(
  DIRECTORY
    launch
    param
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    dynamixel_node
    dynamixel_ros
  DESTINATION
    lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.0.2)
project(mobile_lidar_control)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

## ===== Message/Service Generation =====
## Declare custom messages
# add_message_files(
#   FILES
#   # Add your .msg files here
# )

## Declare custom services
add_service_files(
  FILES
  MoveDistance.srv
)

## Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## ===== Catkin Package Configuration =====
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    message_runtime
)

## ===== Build Configuration =====
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## ===== C++ Library (Uncomment when needed) =====
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/your_source.cpp
# )
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

## ===== C++ Executable Nodes (Uncomment when needed) =====
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## ===== Python Scripts Installation =====
catkin_install_python(PROGRAMS
  scripts/lidar_subscriber.py
  scripts/twist_motion_controller.py
  # scripts/lidar_motion_corrector.py
  # scripts/rviz_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## ===== Install Configuration =====
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


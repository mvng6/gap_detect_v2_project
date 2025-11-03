cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/mobile_robot_twist_control.py
  src/move_mobile_robot_node.py
  src/mobile_robot_control_node.py
  src/battery_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


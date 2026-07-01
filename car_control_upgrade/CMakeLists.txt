cmake_minimum_required(VERSION 3.0.2)
project(car_control_upgrade)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  sbus_serial
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs 
                sensor_msgs diagnostic_msgs visualization_msgs tf sbus_serial
)

## C++ Library: straight_controller_v3
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(straight_controller_v3_node
  src/straight_controller_v3.cpp
)
target_link_libraries(straight_controller_v3_node
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS straight_controller_v3_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Python module: serial_bridge_v2
catkin_install_python(PROGRAMS
  scripts/serial_bridge_v2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

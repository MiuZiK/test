cmake_minimum_required(VERSION 2.8.3)
project(turn_on_wheeltec_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  sbus_serial
)
add_compile_options(-g -std=c++17)  # 建议改为 c++17（支持 std::clamp）

# ulimit -c unlimited
# ssh firefly@192.168.1.76
#add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES turn_on_wheeltec_robot
  #CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf serial sbus_serial
  #DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})

add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})
install(TARGETS wheeltec_robot_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 1. 声明 C++ 节点的可执行文件
add_executable(straight_controller_cpp 
    src/straight_controller.cpp  # 你的 C++ 代码文件路径
)
# 2. 链接依赖库（ROS 核心库 + 消息库）
target_link_libraries(straight_controller_cpp
    ${catkin_LIBRARIES}  # 包含 roscpp, nav_msgs, geometry_msgs, tf 等依赖
)
# 3. 安装 C++ 节点（可选，但推荐，确保 rosrun 能找到）
install(TARGETS straight_controller_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
find_library(WIRINGPI_LIB wiringPi)
target_link_libraries(straight_controller_cpp ${WIRINGPI_LIB})

# Rail Follower Controller (轨道跟随控制器)
add_executable(rail_follower_node 
    src/rail_follower_controller.cpp
)
target_link_libraries(rail_follower_node
    ${catkin_LIBRARIES}
)
install(TARGETS rail_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
target_link_libraries(rail_follower_node ${WIRINGPI_LIB})


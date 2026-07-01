#!/bin/bash
##############################################################################
# 车底巡检小车 - 快速启动（无交互版）
#
# 适用于：
#   - 已熟悉流程，需要快速启动
#   - 通过 SSH 远程启动
#
# 使用方法：
#   ./car_quick_start.sh
##############################################################################

YUNTAI_WS="/home/orangepi/yuntai_ws"
CATKIN_WS="/home/orangepi/catkin_ws"
RADAR_SCRIPT="/home/orangepi/lan_planner/start_scripts/odom.sh"

echo "[START] Car Bot Quick Launch..."

# 预加载环境
source /opt/ros/noetic/setup.bash
[ -f "$CATKIN_WS/devel/setup.bash" ] && source "$CATKIN_WS/devel/setup.bash"
[ -f "$YUNTAI_WS/devel/setup.bash" ] && source "$YUNTAI_WS/devel/setup.bash"

# 启动 FastLIO（终端1）
gnome-terminal -- bash -c "
    echo '[FastLIO] Starting...';
    if [ -f '$RADAR_SCRIPT' ]; then
        cd $(dirname '$RADAR_SCRIPT');
        '$RADAR_SCRIPT';
    else
        echo '[ERROR] Radar script not found';
        sleep 5;
    fi;
    exec bash" &
sleep 3

# 启动云台（终端2）
if [ -d "$YUNTAI_WS" ]; then
    gnome-terminal -- bash -c "
        cd $YUNTAI_WS;
        rosrun yuntai yuntai;
        exec bash" &
    sleep 2
fi

# 启动控制器（终端3）
gnome-terminal -- bash -c "
    cd $CATKIN_WS;
    rosrun turn_on_wheeltec_robot straight_controller_cpp;
    exec bash" &
sleep 2

# 启动车体（终端4）
gnome-terminal -- bash -c "
    cd $CATKIN_WS;
    roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch;
    exec bash" &
sleep 2

# 启动SBUS（终端5）
gnome-terminal -- bash -c "
    cd $CATKIN_WS;
    roslaunch sbus_serial sbus_cmd_vel.launch;
    exec bash" &
sleep 2

# 启动 ROSBridge（终端6）
gnome-terminal -- bash -c "
    cd $CATKIN_WS;
    rosrun turn_on_wheeltec_robot rosbridge_server.py;
    exec bash" &

echo "[DONE] All terminals launched!"
echo "[INFO] IP: $(hostname -I | awk '{print $1}') Port: 1955"
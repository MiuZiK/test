#!/bin/bash
##############################################################################
# 车底巡检小车 - 可视化启动脚本 v3.0
# 
# 特点：
#   ✅ 每个节点在独立终端窗口中运行（gnome-terminal）
#   ✅ 实时查看每个节点的输出日志
#   ✅ 可单独关闭 fastlio 终端以保存 PCD 文件
#   ✅ 支持 Ctrl+C 优雅退出（自动保存 PCD）
#
# 使用方法：
#   chmod +x car_auto_visual.sh
#   ./car_auto_visual.sh
#
# 终端说明：
#   Terminal 1: FastLIO 建图（⭐ 重要！Ctrl+C 可保存 PCD）
#   Terminal 2: 云台控制（90°朝上）
#   Terminal 3: 小车主控制器（straight_controller_cpp）
#   Terminal 4: 车体底层驱动（turn_on_wheeltec_robot.launch）
#   Terminal 5: SBUS 遥控接收
#   Terminal 6: ROSBridge 服务器（Windows Aviateur 连接用）
##############################################################################

# ==================== 配置区 ====================
YUNTAI_WS="/home/orangepi/yuntai_ws"
CATKIN_WS="/home/orangepi/catkin_ws"
RADAR_SCRIPT="/home/orangepi/lan_planner/start_scripts/odom.sh"
LAN_PLANNER_DIR="/home/orangepi/lan_planner"

# 颜色定义（终端标题用）
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==================== 辅助函数 ====================
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}  Car Bot v3.0 - Visual Startup${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_step() {
    echo -e "[${YELLOW}STEP $1${NC}] $2"
}

launch_terminal() {
    local title="$1"
    local cmd="$2"
    local delay="${3:-2}"  # 默认延迟2秒
    
    print_step "Launching" "$title"
    
    gnome-terminal --title="$title" -- bash -c "
        echo '========================================';
        echo '  $title';
        echo '========================================';
        echo '';
        echo '[INFO] Starting... (PID: $$)';
        echo '[INFO] Press Ctrl+C to stop this node';
        echo '';
        $cmd;
        exec bash" &
    
    sleep $delay
}

# ==================== 主流程 ====================
print_header

echo -e "${YELLOW}[配置检查]${NC}"
echo "  Yuntai WS:  $YUNTAI_WS"
echo "  Catkin WS:  $CATKIN_WS"
echo "  Radar Script: $RADAR_SCRIPT"
echo ""

# 检查工作空间是否存在
if [ ! -d "$CATKIN_WS" ]; then
    echo -e "${RED}[ERROR] Catkin workspace not found: $CATKIN_WS${NC}"
    exit 1
fi

if [ ! -d "$YUNTAI_WS" ]; then
    echo -e "${RED}[WARN] Yuntai workspace not found: $YUNTAI_WS (skipping gimbal)${NC}"
fi

echo ""
echo -e "${GREEN}[启动节点] 正在打开终端窗口...${NC}"
echo ""

# ====== Terminal 1: FastLIO 建图（最重要！）======
# ⭐ 这个终端需要保持打开，Ctrl+C 可触发 PCD 保存
launch_terminal "⭐ FastLIO-建图(保存PCD)" "
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    if [ -f '$RADAR_SCRIPT' ]; then
        cd $(dirname '$RADAR_SCRIPT');
        chmod +x '$RADAR_SCRIPT';
        echo '[FASTLIO] Starting radar/odom script...';
        echo '[FASTLIO] ⚠️  按 Ctrl+C 可保存 PCD 文件！';
        '$RADAR_SCRIPT';
    else
        echo '[ERROR] Radar script not found: $RADAR_SCRIPT';
    fi
" 3

# ====== Terminal 2: 云台控制 ======
if [ -d "$YUNTAI_WS" ]; then
    launch_terminal "云台-90°朝上" "
        source /opt/ros/noetic/setup.bash;
        source $YUNTAI_WS/devel/setup.bash;
        cd $YUNTAI_WS;
        echo '[GIMBAL] Starting yuntai node (locked at 90° up)...';
        rosrun yuntai yuntai;
    "
else
    echo -e "${YELLOW}[SKIP] 云台 - 工作空间不存在${NC}"
fi

# ====== Terminal 3: 小车主控制器 ======
launch_terminal "控制器-STANDBY模式" "
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    cd $CATKIN_WS;
    echo '[CONTROLLER] Starting straight_controller_cpp...';
    echo '[CONTROLLER] 默认 STANDBY 模式（上电不会自动走）';
    echo '[CONTROLLER] 使用遥控器 CH6 启动自动巡检';
    rosrun turn_on_wheeltec_robot straight_controller_cpp;
"

# ====== Terminal 4: 车体底层驱动 ======
launch_terminal "车体-底层驱动" "
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    cd $CATKIN_WS;
    echo '[CAR_BODY] Starting turn_on_wheeltec_robot.launch...';
    roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch;
"

# ====== Terminal 5: SBUS 遥控接收 ======
launch_terminal "SBUS-遥控接收" "
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    cd $CATKIN_WS;
    echo '[SBUS] Starting sbus_cmd_vel.launch...';
    roslaunch sbus_serial sbus_cmd_vel.launch;
"

# ====== Terminal 6: ROSBridge 服务器（Aviateur连接）======
launch_terminal "ROSBridge-1955端口" `
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    cd $CATKIN_WS;
    echo '[ROSBridge] Starting rosbridge_server.py on port 1955...';
    echo '[ROSBridge] Windows Aviateur 可通过此端口连接';
    rosrun turn_on_wheeltec_robot rosbridge_server.py;
`

# ==================== 启动完成提示 ====================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ✅ 所有终端已启动完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}📋 终端列表：${NC}"
echo "  1. ⭐ ${RED}FastLIO 建图${NC}       - ${RED}按 Ctrl+C 保存 PCD 文件！${NC}"
echo "  2. 云台控制              - 锁定 90° 朝上"
echo "  3. 控制器                - STANDBY 模式"
echo "  4. 车体底层驱动          - 电机/串口通信"
echo "  5. SBUS 遥控             - 接收遥控器信号"
echo "  6. ROSBridge             - 端口 1955 (Aviateur)"
echo ""
echo -e "${BLUE}🎮 遥控器操作：${NC}"
echo "  CH6 (SW B) UP    → 启动自动巡检"
echo "  CH6 (SW B) DOWN  → 急停"
echo "  CH1/CH2 摇杆     → 手动遥控"
echo ""
echo -e "${YELLOW}💾 保存 PCD 方法：${NC}"
echo "  1. 点击 FastLIO 终端窗口"
echo "  2. 按 Ctrl+C"
echo "  3. 等待 PCD 保存完成（约 2-5 秒）"
echo "  4. 该终端会自动关闭或显示保存路径"
echo ""
echo -e "${GREEN}🌐 Windows Aviateur 连接：${NC}"
echo "  IP: $(hostname -I | awk '{print $1}')"
echo "  Port: 1955"
echo ""
echo -e "${BLUE}========================================${NC}"

# 保持主脚本运行（可选：监控节点状态）
echo -e "[主脚本] 按 Enter 退出（仅关闭此窗口，不影响其他终端）"
read -p ""
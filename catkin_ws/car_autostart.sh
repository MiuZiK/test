#!/bin/bash
##############################################################################
# 车底巡检小车启动脚本 v2.0
# 改进：加速启动 + 遥控器安全控制 + 手动/自动模式分离
#
# 遥控器操作说明：
#   CH6 (SW B) UP   -> 启动自动巡检（上电后默认待机不动）
#   CH6 (SW B) DOWN -> 急停（任何时刻可用）
#   CH6 (SW B) UP   -> 运行中再拨=急停回待机
#   CH1/CH2 摇杆   -> 切入手动遥控（松杆自动恢复）
#   CH5 (SW A)      -> 自动模式下触发返航
#   CH10            -> LED开关
##############################################################################

# ========================== 1. 配置 ==========================
YUNTAI_WS="/home/orangepi/yuntai_ws"
CATKIN_WS="/home/orangepi/catkin_ws"
RADAR_SCRIPT="/home/orangepi/lan_planner/start_scripts/odom.sh"
LOG_FILE="/var/log/car_autostart_v2.log"

# PID文件（用于进程管理）
PID_DIR="/tmp/car_bot_pids"
mkdir -p $PID_DIR

# 最小延迟（仅等待串口初始化）
MIN_DELAY=1

# ========================== 2. 辅助函数 ==========================
log_info()  { echo "[$(date '+%H:%M:%S')] [INFO]  $1" | tee -a $LOG_FILE; }
log_warn()  { echo "[$(date '+%H:%M:%S')] [WARN]  $1" | tee -a $LOG_FILE; }
log_error() { echo "[$(date '+%H:%M:%S')] [ERROR] $1" | tee -a $LOG_FILE; }

# 清理函数：退出时杀掉所有子进程
cleanup() {
    log_warn "Cleaning up all processes..."
    for pidfile in $PID_DIR/*.pid; do
        if [ -f "$pidfile" ]; then
            pid=$(cat "$pidfile" 2>/dev/null)
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill "$pid" 2>/dev/null
                wait "$pid" 2>/dev/null
            fi
            rm -f "$pidfile"
        fi
    done
    log_info "Cleanup complete"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 后台启动并记录PID
launch_bg() {
    local name="$1"
    shift
    local cmd="$@"
    eval "$cmd" >>$LOG_FILE 2>&1 &
    local pid=$!
    echo $pid > "$PID_DIR/${name}.pid"
    echo $pid
}

# 检查进程是否存活
check_pid() {
    local name="$1"
    local pidfile="$PID_DIR/${name}.pid"
    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile")
        kill -0 "$pid" 2>/dev/null
        return $?
    fi
    return 1
}

# ========================== 3. 主流程 ==========================
> $LOG_FILE
log_info "========================================"
log_info " Car Bot v2.0 Startup Script"
log_info "========================================"

# 预加载环境变量
[ -f "$CATKIN_WS/devel/setup.bash" ] && source "$CATKIN_WS/devel/setup.bash"
[ -f "$YUNTAI_WS/devel/setup.bash" ] && source "$YUNTAI_WS/devel/setup.bash"

sleep $MIN_DELAY

# --- 3.1 并行启动云台 ---
log_info "[1/4] Starting gimbal node..."
if [ -d "$YUNTAI_WS" ]; then
    GIMBAL_PID=$(launch_bg "gimbal" "cd $YUNTAI_WS && rosrun yuntai yuntai")
    log_info "  Gimbal PID: $GIMBAL_PID"
else
    log_error "  Yuntai workspace not found: $YUNTAI_WS"
fi

# --- 3.2 并行启动控制节点（v2.0: 上电默认待机，不会自动走）---
log_info "[2/4] Starting controller v2.0 (STANDBY by default)..."
if [ -d "$CATKIN_WS" ]; then
    CTRL_PID=$(launch_bg "controller" "cd $CATKIN_WS && rosrun turn_on_wheeltec_robot straight_controller_cpp")
    log_info "  Controller PID: $CTRL_PID"
else
    log_error "  Catkin workspace not found: $CATKIN_WS"
fi

# --- 3.3 启动车体Launch ---
log_info "[3/4] Starting car body launch..."
CAR_PID=$(launch_bg "car_launch" "cd $CATKIN_WS && roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch")
log_info "  Car launch PID: $CAR_PID"

# --- 3.4 启动SBUS遥控接收 ---
log_info "[4/4] Starting SBUS remote receiver..."
SBUS_PID=$(launch_bg "sbus" "cd $CATKIN_WS && roslaunch sbus_serial sbus_cmd_vel.launch")
log_info "  SBUS PID: $SBUS_PID"

# 短暂等待所有节点初始化完成
sleep 2

# --- 3.5 可选：启动雷达脚本 ---
log_info "[opt] Checking radar script..."
if [ -f "$RADAR_SCRIPT" ]; then
    chmod +x "$RADAR_SCRIPT" 2>/dev/null
    RADAR_PID=$(launch_bg "radar" "$RADAR_SCRIPT")
    log_info "  Radar PID: $RADAR_PID"
else
    log_warn "  Radar script not found: $RADAR_SCRIPT (skipping, non-critical)"
fi

# ========================== 4. 健康检查 & 完成 ==========================
sleep 1
log_info "========================================"
log_info " Startup complete! Health check:"
log_info "========================================"

ALL_OK=true
for name in gimbal controller car_launch sbus; do
    if check_pid $name; then
        pid=$(cat "$PID_DIR/${name}.pid")
        log_info "  [OK] $name (PID=$pid)"
    else
        log_error "  [FAIL] $name - process died!"
        ALL_OK=false
    fi
done

if [ -f "$PID_DIR/radar.pid" ]; then
    if check_pid radar; then
        log_info "  [OK] radar (PID=$(cat $PID_DIR/radar.pid))"
    else
        log_warn "  [WARN] radar - not running (non-critical)"
    fi
fi

log_info "========================================"
if $ALL_OK; then
    log_info " All systems ready!"
    log_info ""
    log_info " Remote control:"
    log_info "   CH6 (SW B) UP   -> START auto inspection"
    log_info "   CH6 (SW B) DOWN -> EMERGENCY STOP"
    log_info "   CH1/CH2 sticks  -> MANUAL mode"
    log_info "   Gimbal locked at 90 deg (up)"
else
    log_warn " Some processes failed! Check $LOG_FILE"
fi
log_info "========================================"
log_info " Log file: $LOG_FILE"
log_info " Press Ctrl+C to stop all processes"

# 保持脚本运行，等待退出信号
wait

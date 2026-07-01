#!/bin/bash
##############################################################################
# 车底巡检小车 - 优雅停止脚本
#
# 功能：
#   ✅ 安全停止所有节点（优先保存 PCD）
#   ✅ 发送 SIGINT 信号触发 FastLIO 优雅退出
#   ✅ 按顺序关闭：FastLIO → 控制器 → 车体 → 其他
#
# 使用方法：
#   chmod +x car_stop.sh
#   ./car_stop.sh
##############################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${YELLOW}  Car Bot - 优雅停止脚本${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# ==================== 1. 查找进程 ====================
echo -e "${GREEN}[1/4] 正在查找运行中的节点...${NC}"

# 定义要查找的进程关键字
declare -A PROCESSES=(
    ["FastLIO"]="odom.sh|fastlio|lins_fusion"
    ["控制器"]="straight_controller_cpp"
    ["云台"]="yuntai"
    ["车体"]="turn_on_wheeltec_robot.launch"
    ["SBUS"]="sbus_cmd_vel"
    ["ROSBridge"]="rosbridge_server.py"
)

# 存储找到的 PID
declare -A PIDS

for name in "${!PROCESSES[@]}"; do
    pattern="${PROCESSES[$name]}"
    pid=$(pgrep -f "$pattern" | head -1)
    
    if [ -n "$pid" ]; then
        PIDS["$name"]="$pid"
        echo -e "  [FOUND] ${GREEN}$name${NC} (PID: $pid)"
    else
        echo -e "  [---]   $name (not running)"
    fi
done

echo ""

# 检查是否有进程在运行
if [ ${#PIDS[@]} -eq 0 ]; then
    echo -e "${YELLOW}[WARN] 没有找到任何运行中的节点${NC}"
    exit 0
fi

# ==================== 2. 确认停止 ====================
echo -e "${YELLOW}即将停止以上 ${#PIDS[@]} 个进程${NC}"
read -p "确认？(y/N): " confirm

if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}已取消操作${NC}"
    exit 0
fi

echo ""
echo -e "${GREEN}[2/4] 开始优雅停止...${NC}"

# ==================== 3. 按顺序停止（优先保存 PCD）==========

# Step 1: 停止 FastLIO（最重要！触发 PCD 保存）
if [[ -v PIDS["FastLIO"] ]]; then
    pid=${PIDS["FastLIO"]}
    echo -e ""
    echo -e "${RED}⭐ [重要] 正在停止 FastLIO (PID: $pid)...${NC}"
    echo -e "${RED}⭐ 这将触发 PCD 文件保存！请等待完成...${NC}"
    
    # 发送 SIGINT（Ctrl+C），让 FastLIO 执行清理和保存
    kill -SIGINT "$pid" 2>/dev/null
    
    # 等待 PCD 保存（最多 10 秒）
    for i in {1..10}; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo -e "${GREEN}  ✓ FastLIO 已退出，PCD 应已保存${NC}"
            break
        fi
        sleep 1
        echo -e "  ⏳ 等待 PCD 保存... ($i/10)"
    done
    
    # 如果还没退出，强制杀死
    if kill -0 "$pid" 2>/dev/null; then
        echo -e "${YELLOW}  ⚠️ FastLIO 未响应，强制终止...${NC}"
        kill -9 "$pid" 2>/dev/null
    fi
fi

sleep 1

# Step 2: 停止控制器
if [[ -v PIDS["控制器"] ]]; then
    pid=${PIDS["控制器"]}
    echo -e "${YELLOW}[3/4] 停止控制器 (PID: $pid)...${NC}"
    kill -SIGTERM "$pid" 2>/dev/null
    sleep 1
fi

# Step 3: 停止车体底层
if [[ -v PIDS["车体"] ]]; then
    pid=${PIDS["车体"]}
    echo -e "${YELLOW}       停止车体驱动 (PID: $pid)...${NC}"
    kill -SIGTERM "$pid" 2>/dev/null
    sleep 1
fi

# Step 4: 停止其他节点
for name in "云台" "SBUS" "ROSBridge"; do
    if [[ -v PIDS["$name"] ]]; then
        pid=${PIDS["$name"]}
        echo -e "${YELLOW}       停止 $name (PID: $pid)...${NC}"
        kill -SIGTERM "$pid" 2>/dev/null
        sleep 0.5
    fi
done

# ==================== 4. 清理残留进程 ================
echo ""
echo -e "${GREEN}[4/4] 清理残留进程...${NC}"

# 强制杀死所有相关进程（如果还在运行）
for pattern in "straight_controller" "yuntai" "turn_on_wheeltec_robot" "sbus_cmd_vel" "rosbridge_server"; do
    pids=$(pgrep -f "$pattern")
    if [ -n "$pids" ]; then
        for pid in $pids; do
            kill -9 "$pid" 2>/dev/null && echo -e "  🔴 Force killed (PID: $pid)"
        done
    fi
done

# ==================== 完成 ===================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ✅ 所有节点已停止！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}📁 PCD 文件位置：${NC}"
echo "  通常保存在：~/lan_planner/ 或 ~/maps/"
echo "  文件名格式：*.pcd 或 *.ply"
echo ""
echo -e "${YELLOW}💡 提示：${NC}"
echo "  - 使用 ls -lh ~/lan_planner/*.pcd 查看文件"
echo "  - 使用 pcl_viewer xxx.pcd 查看 PCD 内容"
echo ""
# 车底巡检小车 - 启动脚本说明文档

## 📦 脚本文件清单

| 脚本名 | 用途 | 推荐场景 |
|--------|------|---------|
| `car_auto_visual.sh` | **可视化启动**（6个独立终端） | ⭐ **日常开发调试** |
| `car_quick_start.sh` | 快速启动（无交互提示） | SSH 远程启动 / 熟练用户 |
| `car_stop.sh` | 优雅停止（优先保存PCD） | 建图完成后停止 |
| `car_autostart.sh` | 后台启动（rc.local 自动启动） | 开机自动运行 |

---

## 🚀 使用方法

### 方法 1：可视化启动（推荐）

```bash
# 1. 赋予执行权限
chmod +x car_auto_visual.sh car_stop.sh car_quick_start.sh

# 2. 运行启动脚本
./car_auto_visual.sh
```

**效果：**
- ✅ 打开 **6 个独立的终端窗口**
- ✅ 每个节点输出清晰可见
- ✅ FastLIO 终端可单独 Ctrl+C 保存 PCD

**终端布局：**

```
┌─────────────────┬─────────────────┬─────────────────┐
│ Terminal 1      │ Terminal 2      │ Terminal 3      │
│ ⭐FastLIO建图   │ 云台-90°朝上    │ 控制器-STANDBY  │
│ (Ctrl+C保存PCD) │                 │                 │
├─────────────────┼─────────────────┼─────────────────┤
│ Terminal 4      │ Terminal 5      │ Terminal 6      │
│ 车体底层驱动     │ SBUS遥控接收    │ ROSBridge:1955  │
│                 │                 │ (Aviateur连接)  │
└─────────────────┴─────────────────┴─────────────────┘
```

---

### 方法 2：快速启动（SSH 远程）

```bash
./car_quick_start.sh
```

**特点：**
- 无交互提示，直接启动
- 适合 SSH 远程登录后快速部署

---

### 方法 3：优雅停止（保存 PCD）

```bash
./car_stop.sh
```

**执行流程：**
1. 显示所有运行中的进程列表
2. 确认是否停止 (y/N)
3. **按顺序停止：**
   - ⭐ **Step 1:** 发送 SIGINT 给 FastLIO → 触发 PCD 保存
   - **Step 2:** 停止控制器
   - **Step 3:** 停止车体底层
   - **Step 4:** 停止其他节点（云台、SBUS、ROSBridge）
4. 清理残留进程
5. 显示 PCD 文件位置

---

## 💾 PCD 文件保存方法

### 方案 A：手动保存（推荐）

```bash
# 1. 在 FastLIO 终端窗口中按 Ctrl+C
# 2. 等待 2-5 秒，看到类似输出：
#    [INFO] Saving map to: /path/to/map.pcd
#    [DONE] Map saved successfully!
# 3. 该终端会自动关闭或显示保存路径
```

### 方案 B：使用 car_stop.sh 脚本

```bash
./car_stop.sh
# 输入 y 确认
# 脚本会自动发送 SIGINT 给 FastLIO 并等待 PCD 保存完成
```

### 方案 C：命令行直接发送信号

```bash
# 1. 找到 FastLIO 进程 PID
pgrep -f "odom.sh\|fastlio"

# 2. 发送 SIGINT（模拟 Ctrl+C）
kill -SIGINT <PID>

# 3. 等待保存完成
sleep 5

# 4. 验证文件是否存在
ls -lh ~/lan_planner/*.pcd
```

---

## 🔧 配置修改

### 修改 IP 地址或端口

编辑 `car_auto_visual.sh`：

```bash
# 第 15-18 行
YUNTAI_WS="/home/orangepi/yuntai_ws"        # 云台工作空间
CATKIN_WS="/home/orangepi/catkin_ws"         # ROS 工作空间
RADAR_SCRIPT="/home/orangepi/lan_planner/start_scripts/odom.sh"  # FastLIO 脚本
LAN_PLANNER_DIR="/home/orangepi/lan_planner" # lan_planner 目录
```

### 添加新节点

在 `car_auto_visual.sh` 的主流程中添加：

```bash
# 示例：添加一个新的终端
launch_terminal "新节点名称" "
    source /opt/ros/noetic/setup.bash;
    source $CATKIN_WS/devel/setup.bash;
    # 你的启动命令
    rosrun package_name node_name;
"
```

---

## 🎮 遥控器操作说明

| 操作 | 遥控器动作 | 效果 |
|------|-----------|------|
| 启动自动巡检 | CH6 (SW B) ↑ UP | 小车开始直线巡检 |
| 急停 | CH6 (SW B) ↓ DOWN | 立即停车 |
| 手动控制 | CH1/CH2 摇杆 | 手动遥控（松杆恢复自动） |
| 返航 | CH5 (SW A) | 自动模式下触发返航 |
| LED开关 | CH10 | 切换 LED 状态 |

**注意：** 上电后默认 **STANDBY 模式**，小车不会自动走！

---

## 🌐 Windows Aviateur 连接

### 启动步骤：

1. **在小车上启动 ROSBridge：**
   ```bash
   ./car_auto_visual.sh  # Terminal 6 会自动启动 ROSBridge
   ```

2. **在 Windows 上启动 Aviateur：**
   ```powershell
   cd C:\dev\aviateur_build\aviateur\build\bin\Debug
   .\aviateur.exe
   ```

3. **在 Aviateur 中连接：**
   - 点击 "Car Control" 选项卡
   - 输入小车的 IP 地址（如 `192.168.11.119`）
   - 点击 "Connect"

4. **测试控制：**
   - 使用方向键按钮控制小车
   - 点击模式切换按钮改变运行模式

### 连接参数：

| 参数 | 值 |
|------|-----|
| 协议 | TCP Socket |
| 端口 | 1955 |
| 数据格式 | JSON (4字节长度前缀 + JSON体) |

---

## ❓ 常见问题

### Q1: 终端窗口没有打开？
**A:** 检查是否安装了 gnome-terminal：
```bash
which gnome-terminal
# 如果没有，安装：
sudo apt install gnome-terminal
```

### Q2: FastLIO 启动失败？
**A:** 检查雷达脚本是否存在：
```bash
ls -la /home/orangepi/lan_planner/start_scripts/odom.sh
# 如果不存在，检查路径配置
```

### Q3: PCD 文件没有保存？
**A:** 确保 FastLIO 收到的是 **SIGINT** (Ctrl+C)，而不是 SIGKILL (-9)

### Q4: 如何查看已保存的 PCD 文件？
```bash
# 查找所有 pcd 文件
find ~/lan_planner -name "*.pcd" -o -name "*.ply"

# 使用 PCL 查看器打开
pcl_viewer xxx.pcd
```

### Q5: rc.local 自动启动如何切换到可视化模式？

**方案 A：注释掉 rc.local 中的自动启动**
```bash
sudo nano /etc/rc.local
# 注释掉这行：
# /home/orangepi/lan_planner/start_scripts/car_autostart.sh &
```

**方案 B：保留两者，开机后手动启动可视化版本**
- 开机自动启动后台版本（基本功能）
- SSH 登录后运行 `./car_auto_visual.sh`（先 stop 再 start）

---

## 📊 对比：旧版 vs 新版

| 特性 | car_autostart.sh (旧) | car_auto_visual.sh (新) |
|------|---------------------|------------------------|
| 启动方式 | 后台运行 | 独立终端窗口 |
| 日志查看 | 查看 /var/log/xxx.log | 直接在终端看实时输出 |
| PCD 保存 | 困难（需要 kill 进程） | ⭐ **简单（Ctrl+C 即可）** |
| 调试方便性 | 低 | ⭐ **高** |
| 适用场景 | 开机自动运行 | 开发调试 / 建图任务 |

---

## 📝 更新日志

### v3.0 (2026-05-08)
- ✅ 新增可视化启动脚本 `car_auto_visual.sh`
- ✅ 新增优雅停止脚本 `car_stop.sh`
- ✅ 新增快速启动脚本 `car_quick_start.sh`
- ✅ 支持 FastLIO PCD 优雅保存
- ✅ 集成 ROSBridge 服务器启动
- ✅ 添加详细的使用说明文档

### v2.0 (之前)
- 后台启动方式
- PID 管理
- 日志记录到文件

---

## 🎯 推荐工作流程

### 日常开发调试：
```bash
./car_auto_visual.sh    # 可视化启动
# ... 测试 ...
# 在 FastLIO 终端按 Ctrl+C 保存 PCD
./car_stop.sh          # 或者用脚本优雅停止
```

### SSH 远程部署：
```bash
ssh orangepi@<IP>
cd ~/catkin_ws
./car_quick_start.sh   # 快速启动
exit                   # 断开 SSH（节点继续运行）
```

### 开机自动运行（保持）：
```bash
# 保持 /etc/rc.local 中的配置
sudo nano /etc/rc.local
# 确保有这一行：
/home/orangepi/lan_planner/start_scripts/car_autostart.sh &
```

---

**🎉 现在你可以享受更便捷的车辆控制和 PCD 保存体验了！**
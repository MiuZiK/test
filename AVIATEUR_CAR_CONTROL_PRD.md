# Aviateur 小车控制集成模块 - 产品需求文档 (PRD)

## 📋 文档信息

| 项目 | 内容 |
|------|------|
| **产品名称** | Aviateur Undercarriage Control Module |
| **版本** | v1.0 |
| **基于** | OpenIPC/aviateur v0.1.2 |
| **目标平台** | Windows (优先), Linux |
| **创建日期** | 2026-05-06 |
| **状态** | 设计阶段 |

---

## 1. 项目背景与目标

### 1.1 现状分析

**Aviateur 当前能力：**
- ✅ FPV视频接收与实时显示
- ✅ 截图(JPEG) / 录制(GIF/MP4)
- ✅ Wi-Fi信号质量监测
- ✅ 多种解码器支持(FFmpeg/GStreamer)
- ❌ **缺少小车控制功能**
- ❌ **缺少传感器数据显示**
- ❌ **缺少状态监控**

**用户需求：**
> "将之前开发的小车控制Web端功能直接集成到Aviateur软件中，实现同一局域网下连接小车后直接控制和参数显示"

**核心价值：**
```
单一应用 = 视频监控 + 实时控制 + 状态监控
         ↓
提升操作效率，降低切换成本
```

---

## 2. 功能需求

### 2.1 核心功能清单

#### 🔴 **P0 - 必须实现 (MVP)**

| ID | 功能 | 描述 | 优先级 |
|----|------|------|--------|
| F01 | **网络连接管理** | 输入小车IP地址，建立TCP/WebSocket连接 | P0 |
| F02 | **虚拟摇杆控制** | 屏幕触摸/鼠标模拟遥控器双轴摇杆 | P0 |
| F03 | **速度指令发送** | 发送 /cmd_vel (linear.x, angular.z) | P0 |
| F04 | **模式切换** | STANDBY/AUTO/MANUAL 模式切换按钮 | P0 |
| F05 | **急停按钮** | 一键紧急停止（醒目红色大按钮） | P0 |
| F06 | **连接状态指示** | 已连接/未连接/错误 状态灯 | P0 |

#### 🟡 **P1 - 重要功能 (V1.1)**

| ID | 功能 | 描述 | 优先级 |
|----|------|------|--------|
| F07 | **实时状态显示** | 显示当前模式、行驶距离、航向角 | P1 |
| F08 | **传感器数据面板** | Odometry位置(x,y)、速度、电池电压 | P1 |
| F09 | **轨道检测可视化** | 在视频上叠加轨道线检测结果 | P1 |
| F10 | **云台控制** | 云台角度调节滑块(俯仰/偏航) | P1 |
| F11 | **LED控制** | LED开关按钮 | P1 |
| F12 | **日志输出窗口** | 实时显示ROS话题消息和控制指令 | P1 |

#### 🟢 **P2 - 增强功能 (V2.0)**

| ID | 功能 | 描述 | 优先级 |
|----|------|------|--------|
| F13 | **参数配置面板** | RANSAC阈值、速度、PID系数等可调参数 | P2 |
| F14 | **历史轨迹回放** | 绘制已行驶路径（在OSD或独立窗口） | P2 |
| F15 | **多车支持** | 同时连接多辆小车（标签页切换） | P2 |
| F16 | **录制同步数据** | 录制视频同时记录控制指令和时间戳 | P2 |
| F17 | **键盘快捷键** | WASD方向控制，空格急停等 | P2 |

---

## 3. 技术架构设计

### 3.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                     Aviateur 主程序                          │
│  ┌──────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │ 视频播放  │  │  原有控制面板  │  │  🆕 小车控制面板     │   │
│  │ (Player) │  │ (ControlPanel)│  │  (CarControlPanel)  │   │
│  └────┬─────┘  └──────┬───────┘  └──────────┬──────────┘   │
│       │               │                      │              │
├───────┼───────────────┼──────────────────────┼──────────────┤
│       ▼               ▼                      ▼              │
│  ┌──────────────────────────────────────────────────┐      │
│  │              🆕 ROS Bridge Module                │      │
│  │  ┌─────────────┐  ┌────────────┐  ┌──────────┐  │      │
│  │  │TCP Client   │  │Protocol    │  │Message   │  │      │
│  │  │(Async I/O)  │  │Handler    │  │Queue     │  │      │
│  │  └──────┬──────┘  └─────┬────┘  └─────┬────┘  │      │
│  └─────────┼───────────────┼─────────────┼────────┘      │
│            │               │             │                │
└────────────┼───────────────┼─────────────┼────────────────┘
             │               │             │
             ▼               ▼             ▼
    ┌─────────────────────────────────────────────┐
    │            网络 (LAN/WiFi)                  │
    │         TCP Port: 1955 (自定义)             │
    └─────────────────┬───────────────────────────┘
                      │
                      ▼
    ┌─────────────────────────────────────────────┐
    │           小车 ROS 系统                       │
    │  ┌──────────┐  ┌───────────┐  ┌──────────┐  │
    │  │rosbridge │  │/cmd_vel   │  │/Odometry │  │
    │  │Server    │  │Subscriber │  │Publisher │  │
    │  └──────────┘  └───────────┘  └──────────┘  │
    └─────────────────────────────────────────────┘
```

### 3.2 通信协议选择

**方案对比：**

| 方案 | 优点 | 缺点 | 适用性 |
|------|------|------|--------|
| **rosbridge_suite** | 标准ROS方案，功能全 | 需要额外安装，依赖重 | ⭐⭐⭐ |
| **自定义TCP + JSON** | 轻量，易实现，无依赖 | 需自己定义协议 | ⭐⭐⭐⭐⭐ |
| **WebSocket** | 双向通信，浏览器兼容 | C++需额外库 | ⭐⭐⭐ |
| **UDP** | 低延迟 | 不可靠，需自己处理丢包 | ⭐⭐ |

**推荐：自定义TCP + JSON协议**
- ✅ 轻量级（单文件~500行）
- ✅ 无外部依赖（只需标准socket）
- ✅ 易于调试（可用telnet测试）
- ✅ 与Aviateur现有架构一致（C++原生）

### 3.3 自定义协议规范

```json
// ====== 客户端 → 小车 (命令) ======

// 1. 速度控制
{
  "type": "cmd_vel",
  "timestamp": 1687890123.456,
  "data": {
    "linear_x": 0.33,
    "angular_z": 0.15
  }
}

// 2. 模式切换
{
  "type": "mode_change",
  "data": {
    "mode": "AUTO"  // STANDBY | AUTO | MANUAL
  }
}

// 3. 急停
{
  "type": "emergency_stop",
  "data": {}
}

// 4. 云台控制
{
  "type": "gimbal_cmd",
  "data": {
    "yaw": 0,      // 偏航角度
    "pitch": 90    // 俯仰角度
  }
}

// ====== 小车 → 客户端 (状态) ======

// 1. 系统状态
{
  "type": "system_status",
  "timestamp": 1687890123.456,
  "data": {
    "mode": "AUTO",
    "battery_voltage": 11.8,
    "uptime": 1234.5,
    "connected": true
  }
}

// 2. 里程计数据
{
  "type": "odometry",
  "timestamp": 1687890123.456,
  "data": {
    "position": {"x": 5.23, "y": 0.12},
    "orientation": {"yaw": 1.57},
    "linear_velocity": 0.33,
    "angular_velocity": 0.02
  }
}

// 3. 轨道检测状态
{
  "type": "rail_status",
  "timestamp": 1687890123.456,
  "data": {
    "detected": true,
    "confidence": 0.89,
    "rail_width": 0.64,
    "cross_track_error": 0.02
  }
}
```

---

## 4. UI/UX 设计

### 4.1 界面布局

```
┌──────────────────────────────────────────────────────────────────┐
│  Aviateur - Undercarriage Inspection Control                    │
├──────────────────────────────┬───────────────────────────────────┤
│                              │                                   │
│                              │    📊 状态面板                    │
│                              │    ┌─────────────────────┐      │
│                              │    │ Mode: [AUTO] ●        │      │
│       📹 视频显示区域         │    │ Distance: 12.3m       │      │
│       (原有PlayerRect)       │    │ Position: (5.2, 0.1)   │      │
│                              │    │ Yaw: 89.2°            │      │
│                              │    │ Battery: 11.8V ████░  │      │
│                              │    │ Rail: ✓ Conf: 0.89    │      │
│                              │    └─────────────────────┘      │
│                              │                                   │
│                              │    🎮 控制区域                   │
│                              │    ┌─────────────────────┐      │
│                              │    │      ↑ (Forward)      │      │
│                              │    │   ←  ●  →            │      │
│                              │    │      ↓ (Backward)     │      │
│                              │    └─────────────────────┘      │
│                              │                                   │
│                              │  [🛑 STOP] [MODE] [GIMBAL] [LED] │
│                              │                                   │
├──────────────────────────────┴───────────────────────────────────┤
│  [Adapter] [Channel] [WFB Key] | [Connect] [Record] [Settings]  │
└──────────────────────────────────────────────────────────────────┘
```

### 4.2 UI组件说明

| 组件 | 类型 | 尺寸 | 交互方式 | 说明 |
|------|------|------|---------|------|
| **视频区** | PlayerRect | 60%屏幕 | 只读 | 原有组件，叠加OSD信息 |
| **状态面板** | Panel | 右侧30% | 只读 | 实时更新数据 |
| **虚拟摇杆** | CustomWidget | 200x200px | 鼠标拖拽/触摸 | 发送cmd_vel |
| **急停按钮** | Button | 大号红色 | 点击 | emergency_stop |
| **模式按钮** | ToggleButton | 中等 | 点击 | 切换STANDBY/AUTO/MANUAL |
| **连接输入** | TextInput + Button | 顶部栏 | 输入IP+点击 | 建立TCP连接 |
| **日志窗口** | TextArea | 底部可选 | 滚动只读 | 显示收发消息 |

### 4.3 交互流程

```
启动Aviateur 
    ↓
[设置页] 输入小车IP地址 (默认: 192.168.1.100)
    ↓
点击 [Connect] 按钮
    ↓
┌─ 连接成功? ─┐
│              │
▼  Yes         ▼ No
│              │
│  显示绿色状态灯│  显示红色+错误信息
│  启用控制UI  │  禁用控制UI
│  开始接收状态│  允许重新连接
│              │
▼              ▼
│  [正常使用]   │
│  ├─ 操作摇杆  │
│  │   → 发送cmd_vel
│  │   → 视频观察反馈
│  ├─ 切换模式  │
│  │   → AUTO/MANUAL
│  └─ 点击STOP  │
│      → 急停   │
└──────────────┘
```

---

## 5. 技术实现细节

### 5.1 文件结构（新增/修改）

```
aviateur/
├── src/
│   ├── feature/
│   │   └── car_control/          # 🆕 新增：小车控制功能模块
│   │       ├── CMakeLists.txt
│   │       ├── ros_bridge.h/cpp      # ROS桥接（TCP客户端）
│   │       ├── protocol.h/cpp         # 协议编解码(JSON)
│   │       ├── message_types.h        # 消息类型定义
│   │       └── command_queue.h/cpp    # 命令队列(线程安全)
│   │
│   ├── gui/
│   │   ├── car_control_panel.h/cpp  # 🆕 新增：控制面板UI
│   │   ├── virtual_joystick.h/cpp   # 🆕 新增：虚拟摇杆组件
│   │   ├── status_display.h/cpp     # 🆕 新增：状态显示组件
│   │   ├── control_panel.h/cpp      # 修改：添加连接入口
│   │   └── settings_tab.h/cpp       # 修改：添加小车设置Tab
│   │
│   ├── gui_interface.h             # 修改：添加CarControlConfig结构
│   └── main.cpp                    # 修改：初始化CarControl模块
│
├── assets/
│   └── icons/                     # 🆕 新增：控制相关图标
│       ├── joystick_bg.png
│       ├── joystick_knob.png
│       ├── stop_button.png
│       ├── connect.png
│       └── disconnect.png
│
└── CMakeLists.txt                 # 修改：添加新模块编译选项
```

### 5.2 关键类设计

#### **ROSBridge (ros_bridge.h)**

```cpp
class ROSBridge {
public:
    ROSBridge();
    ~ROSBridge();
    
    // 连接管理
    bool connect(const std::string& ip, int port = 1955);
    void disconnect();
    bool isConnected() const;
    
    // 命令发送
    void sendCmdVel(float linear_x, float angular_z);
    void sendModeChange(const std::string& mode);
    void sendEmergencyStop();
    void sendGimbalCmd(float yaw, float pitch);
    void sendLEDToggle(bool on);
    
    // 回调注册
    using StatusCallback = std::function<void(const SystemStatus&)>;
    using OdomCallback = std::function<void(const OdometryData&)>;
    using RailCallback = std::function<void(const RailStatus&)>;
    
    void onSystemStatus(StatusCallback cb);
    void onOdometry(OdomCallback cb);
    void onRailStatus(RailCallback cb);
    
private:
    void receiveLoop();  // 后台接收线程
    void parseMessage(const std::string& json);
    
    int socket_fd_;
    std::atomic<bool> connected_;
    std::thread recv_thread_;
    
    StatusCallback status_cb_;
    OdomCallback odom_cb_;
    RailCallback rail_cb_;
};
```

#### **VirtualJoystick (virtual_joystick.h)**

```cpp
class VirtualJoystick : public Revector::Widget {
public:
    VirtualJoystick();
    
    // Revector接口
    void render() override;
    void onMouseDown(int x, int y) override;
    void onMouseMove(int x, int y) override;
    void onMouseUp(int x, int y) override;
    
    // 数据获取
    float getLinearX() const;   // [-1, 1]
    float getAngularZ() const;  // [-1, 1]
    bool isActive() const;
    
    // 回调
    using JoystickCallback = std::function<void(float, float)>;
    void onValueChanged(JoystickCallback cb);
    
private:
    float knob_x_, knob_y_;     // 摇杆 knob位置
    float center_x_, center_y_; // 中心点
    float radius_;              // 摇杆活动半径
    bool is_dragging_;
    
    JoystickCallback value_changed_cb_;
};
```

#### **CarControlPanel (car_control_panel.h)**

```cpp
class CarControlPanel : public Revector::Panel {
public:
    CarControlPanel();
    
    void render() override;
    void update();  // 每帧调用，刷新状态显示
    
    // 事件处理
    void onConnectClicked();
    void onDisconnectClicked();
    void onStopClicked();
    void onModeChanged(const std::string& mode);
    
private:
    // 子组件
    VirtualJoystick* joystick_;
    StatusDisplay* status_display_;
    Revector::Button* stop_btn_;
    Revector::ToggleButton* mode_btn_;
    Revector::TextInput* ip_input_;
    Revector::Label* connection_status_;
    
    // 数据模型
    ROSBridge bridge_;
    SystemStatus current_status_;
    OdometryData current_odom_;
};
```

### 5.3 编译配置 (CMakeLists.txt)

```cmake
# 在主CMakeLists.txt中添加选项
option(ENABLE_CAR_CONTROL "Enable car control module" ON)

if(ENABLE_CAR_CONTROL)
    add_subdirectory(src/feature/car_control)
    target_link_libraries(aviateur PRIVATE car_control)
endif()
```

```cmake
# src/feature/car_control/CMakeLists.txt
set(CAR_CONTROL_SOURCES
    ros_bridge.cpp
    protocol.cpp
    command_queue.cpp
)

set(CAR_CONTROL_HEADERS
    ros_bridge.h
    protocol.h
    message_types.h
    command_queue.h
)

add_library(car_control STATIC 
    ${CAR_CONTROL_SOURCES} ${CAR_CONTROL_HEADERS}
)

target_include_directories(car_control PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/json/single_include  # nlohmann/json
)

target_link_libraries(car_control PRIVATE
    ws2_32  # Windows socket库
    pthread # Linux线程库
)
```

---

## 6. 开发计划

### 6.1 里程碑

| 阶段 | 时间 | 交付物 | 验收标准 |
|------|------|--------|---------|
| **M1: 基础通信** | Day 1-2 | ROSBridge模块 | 能连接小车并收发JSON消息 |
| **M2: UI框架** | Day 3-4 | 控制面板骨架 | 显示面板布局，连接按钮可用 |
| **M3: 核心控制** | Day 5-6 | 虚拟摇杆+急停 | 摇杆能控制小车移动，急停有效 |
| **M4: 状态显示** | Day 7-8 | 状态面板 | 实时显示odometry和系统状态 |
| **M5: 集成测试** | Day 9-10 | 完整集成 | 全功能可用，通过测试用例 |

### 6.2 测试策略

**单元测试：**
- Protocol编解码正确性
- CommandQueue线程安全性
- JSON解析边界情况

**集成测试：**
- 连接/断开重连
- 高频命令发送稳定性
- 长时间运行内存泄漏

**手动验收测试：**
- [ ] 启动软件能看到新增的控制面板
- [ ] 输入IP能成功连接小车
- [ ] 拖动摇杆小车能响应运动
- [ ] 点击急停小车立即停止
- [ ] 模式切换按钮工作正常
- [ ] 状态数据实时更新显示
- [ ] 断开连接后自动禁用控制

---

## 7. 风险与缓解

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|---------|
| 网络延迟导致控制不灵敏 | 高 | 中 | 本地预测+插值，命令队列平滑 |
| Aviateur UI框架学习曲线 | 中 | 低 | 参考现有control_panel代码 |
| ROS端需要配套服务 | 高 | 高 | 提供rosbridge_server.py脚本 |
| 多平台兼容性问题 | 中 | 中 | 使用条件编译(#ifdef) |
| 性能开销影响视频帧率 | 低 | 低 | 控制逻辑放独立线程，不影响渲染 |

---

## 8. 附录

### A. 依赖项清单

**新增依赖：**
- nlohmann/json (已在3rdparty/json)
- Winsock2 (Windows自带)
- POSIX sockets (Linux自带)

**无需新增外部库！** ✅

### B. 参考文件

- Aviateur原项目：https://github.com/OpenIPC/aviateur/tree/0.1.2
- 参考UI实现：`src/gui/control_panel.cpp`
- 参考架构：`src/gui_interface.h`
- ROS桥参考：rosbridge_suite (但我们将用更轻量的自实现)

### C. 术语表

| 术语 | 说明 |
|------|------|
| Aviateur | OpenIPC的FPV地面站软件 |
| cmd_vel | ROS标准速度控制话题(geometry_msgs/Twist) |
| odometry | ROS标准里程计话题(nav_msgs/Odometry) |
| VirtualJoystick | 屏幕上的虚拟摇杆控件 |
| ROSBridge | 本地PC与小车ROS系统的通信桥梁 |

---

**文档版本**: v1.0  
**最后更新**: 2026-05-06  
**作者**: AI Assistant  
**审核人**: [待填写]

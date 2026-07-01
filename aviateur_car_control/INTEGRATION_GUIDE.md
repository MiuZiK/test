# Aviateur 小车控制模块 - 集成与部署指南

## 📋 快速开始

### 前置条件

1. **Aviateur 源码** (v0.1.2)
   ```bash
   git clone -b 0.1.2 https://github.com/OpenIPC/aviateur.git
   ```

2. **小车端 ROS 环境**
   - ROS (Kinetic/Melodic/Noetic)
   - Python 3.x
   - rospy, geometry_msgs, nav_msgs

3. **网络连接**
   - Windows PC (运行Aviateur) 和 小车 在同一局域网
   - 防火墙允许 TCP 1955 端口通信

---

## 🔧 步骤一：集成到 Aviateur (Windows端)

### 1. 复制文件

将 `aviateur_car_control/` 目录下的文件复制到 Aviateur 项目：

```bash
# 假设 Aviateur源码在 D:\dev\aviateur
cp car_control_module.h   D:\dev\aviateur\src\feature\car_control\
cp car_control_module.cpp D:\dev\aviateur\src\feature\car_control\
```

### 2. 修改 CMakeLists.txt

在 `aviateur/CMakeLists.txt` 中添加：

```cmake
# ====== 可选功能: 小车控制模块 ======
option(ENABLE_CAR_CONTROL "Enable undercarriage vehicle control module" ON)

if(ENABLE_CAR_CONTROL)
    add_subdirectory(src/feature/car_control)
    target_link_libraries(aviateur PRIVATE car_control)
    
    # Windows需要链接Winsock
    if(WIN32)
        target_link_libraries(car_control PRIVATE ws2_32)
    endif()
endif()
```

创建文件 `src/feature/car_control/CMakeLists.txt`：

```cmake
set(CAR_CONTROL_SOURCES
    car_control_module.cpp
)

add_library(car_control STATIC ${CAR_CONTROL_SOURCES})

target_include_directories(car_control PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd/json/single_include/nlohmann
)

message(STATUS "Car Control Module: ENABLED")
```

### 3. 修改 gui_interface.h

在 `src/gui_interface.h` 中添加包含和配置结构：

```cpp
// 在文件顶部添加
#ifdef ENABLE_CAR_CONTROL
    #include "feature/car_control/car_control_module.h"
#endif

// 在主配置结构体中添加
struct AppConfig {
    // ... 现有字段 ...
    
    # ====== Car Control (新增) ======
#ifdef ENABLE_CAR_CONTROL
    bool enable_car_control = true;
    std::string car_ip = "192.168.1.100";
    int car_port = 1955;
#endif
};
```

### 4. 修改 main.cpp (初始化)

在 `src/main.cpp` 的初始化部分添加：

```cpp
#ifdef ENABLE_CAR_CONTROL
    #include "feature/car_control/car_control_module.h"
#endif

// 在 main() 函数中，UI初始化之后：
#ifdef ENABLE_CAR_CONTROL
    if (config.enable_car_control) {
        auto* car_panel = new car_control::CarControlPanel();
        car_panel->init();
        car_panel->setBounds(screen_width - 320, 10, 300, screen_height - 20);
        
        // 注册到主窗口的更新循环中
        main_window->addWidget(car_panel);
    }
#endif
```

### 5. 编译

```bash
cd D:\dev\aviateur
mkdir build && cd build
cmake .. -DENABLE_CAR_CONTROL=ON
cmake --build . --config Release
```

编译输出应该在 `build/bin/Release/aviateur.exe`

---

## 🔧 步骤二：部署到小车 (Linux端)

### 1. 复制脚本

将 `rosbridge_server.py` 复制到小车的ROS工作空间：

```bash
scp rosbridge_server.py user@<小车IP>:~/catkin_ws/src/<your_package>/scripts/
```

或直接在小车上创建：

```bash
# 在小车上执行
cat > ~/catkin_ws/src/aviateur_car_control/scripts/rosbridge_server.py << 'EOF'
# (粘贴 rosbridge_server.py 内容)
EOF
chmod +x ~/catkin_ws/src/aviateur_car_control/scripts/rosbridge_server.py
```

### 2. 测试服务器

```bash
# 在小车的终端中运行
rosrun aviateur_car_control rosbridge_server.py --port 1955 --verbose

# 应该看到:
# [Server] Initialized on 0.0.0.0:1955
# [INFO] [rosout]: ...
# [Server] ✓ Listening for connections on 0.0.0.0:1955
```

### 3. (可选) 创建启动文件

创建 `launch/car_bridge.launch`:

```xml
<launch>
    <node name="aviateur_ros_bridge" pkg="aviateur_car_control" 
          type="rosbridge_server.py" output="screen">
        <param name="~port" value="1955"/>
    </node>
</launch>
```

使用方式：
```bash
roslaunch aviateur_car_control car_bridge.launch
```

---

## 🚀 步骤三：联调测试

### 1. 启动顺序

```
┌─────────────────────────────────────────┐
│  小车 (Linux + ROS)                    │
│                                         │
│  ① roscore                             │
│  ② roslaunch ... straight_controller     │
│  ③ rosrun aviateur_car_control           │
│     rosbridge_server.py --port 1955     │
│                                         │
│  ✅ 等待连接...                         │
└──────────────────────┬──────────────────┘
                       │ TCP 1955
                       │
┌──────────────────────▼──────────────────┐
│  地面站 (Windows + Aviateur)            │
│                                         │
│  ④ 运行 aviateur.exe                     │
│  ⑤ 打开 Car Control 面板               │
│  ⑥ 输入小车 IP → 点击 Connect            │
│                                         │
│  ✅ 连接成功! 开始控制                   │
└─────────────────────────────────────────┘
```

### 2. 功能验证清单

- [ ] **连接**: 输入IP后点击Connect，状态灯变绿
- [ ] **虚拟摇杆**: 拖动摇杆，小车应响应移动（观察视频）
- [ ] **急停**: 点击红色STOP按钮，小车立即停止
- [ ] **模式切换**: 点击MODE按钮，终端应显示模式变化
- [ ] **状态显示**: 右侧面板实时更新位置、电池等信息
- [ ] **键盘控制**: WASD键控制方向，空格急停
- [ ] **断开重连**: 断开后重新连接，状态恢复

---

## 🐛 故障排查

### 问题1: 连接失败 "Connection refused"

**原因**: 小车端的rosbridge_server未运行或端口被占用

**解决**:
```bash
# 小车端检查
netstat -tlnp | grep 1955
# 如果没有输出，说明服务未启动

# 手动测试端口
nc -l 1955  # 或 python -m http.server 1955 (临时测试)

# 检查防火墙
sudo ufw allow 1955/tcp
```

### 问题2: 发送指令但小车无反应

**原因**: 
- /cmd_vel 话题名称不匹配
- 底层控制器未订阅该话题
- 权限问题

**解决**:
```bash
# 检查话题列表
rostopic list | grep cmd_vel

# 手动发布测试
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"

# 如果手动发布有效，检查代码中的话题名是否一致
```

### 问题3: 状态数据不更新

**原因**: Odometry话题未发布或话题名不匹配

**解决**:
```bash
# 检查是否有Odometry数据
rostopic echo /Odometry -n 1

# 如果没有，确保FastLIO或其他定位系统正在运行
# 并确认话题名为 /Odometry (不是 /odom 或其他)
```

### 问题4: 视频和控制不同步

**现象**: 控制延迟大，操作与视频反应不一致

**解决**:
- 检查网络延迟: `ping <小车IP>`
- 降低视频码率以节省带宽
- 考虑使用有线以太网代替Wi-Fi

---

## 📊 性能指标

| 指标 | 目标值 | 测试方法 |
|------|--------|---------|
| **连接延迟** | <100ms | 从点击Connect到状态灯变绿 |
| **命令延迟** | <50ms | 从拖动摇杆到小车开始移动 |
| **状态刷新率** | 10Hz | 右侧面板数值更新频率 |
| **CPU开销(Aviateur)** | <5% | 任务管理器监控 |
| **CPU开销(小车)** | <2% | top命令查看rosbridge进程 |

---

## 🔒 安全注意事项

1. **网络安全**: 此协议无加密，仅在可信局域网使用
2. **急停机制**: 必须保证急停按钮醒目且易于触发
3. **权限控制**: 建议添加密码认证（后续版本）
4. **日志记录**: 所有控制指令应记录，便于事故分析

---

## 📝 后续开发建议

### V1.1 计划
- [ ] 添加密码认证
- [ ] 支持多车同时连接
- [ ] 录制控制日志回放
- [ ] 参数配置持久化(INI文件)

### V2.0 计划
- [ ] 视频叠加OSD信息(速度、距离)
- [ ] 轨道检测可视化(在视频上画线)
- [ ] 路径规划预览
- [ ] 远程固件升级

---

**文档版本**: v1.0  
**最后更新**: 2026-05-06  
**适用代码**: aviateur_car_control/

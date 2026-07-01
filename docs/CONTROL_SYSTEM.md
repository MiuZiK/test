# 车辆控制系统

## 目录结构

```
car_control/
├── stm32/                    # STM32固件代码
│   ├── inc/                  # 头文件
│   │   ├── ekf_estimator.h   # EKF状态估计
│   │   ├── pid_controller.h  # PID控制器
│   │   └── wheeltec_protocol_v2.h  # 串口通信协议
│   ├── src/                  # 源文件
│   │   ├── ekf_estimator.c
│   │   ├── pid_controller.c
│   │   └── wheeltec_protocol_v2.c
│   └── balance_v3_integration.c  # 主程序入口
├── ros/                      # ROS节点
│   ├── serial_bridge_v2.py   # 串口桥接节点
│   └── straight_controller_v3.cpp  # 直行控制器
├── launch/                   # ROS启动配置
│   └── control_v3.launch
└── CMakeLists.txt            # CMake构建配置
```

## 功能说明

### 1. SBUS串口通信

**文件**: `catkin_ws/src/sbus_serial/`

- 接收遥控器SBUS信号
- 解析16通道PWM数据
- 发布ROS消息 `Sbus.msg`
- 支持命令速度转换

### 2. 直行控制器

**文件**: `car_control/ros/straight_controller_v3.cpp`

- 读取SBUS输入
- 计算速度和转向控制
- 发布控制命令

### 3. 串口桥接

**文件**: `car_control/ros/serial_bridge_v2.py`

- 连接ROS与STM32
- 转发控制指令
- 接收状态反馈

### 4. STM32固件

**文件**: `car_control/stm32/`

- PID控制器：速度和转向控制
- EKF状态估计：融合传感器数据
- Wheeltec协议：与ROS通信

## 使用方法

### 启动ROS节点

```bash
# 启动SBUS串口节点
roslaunch sbus_serial sbus_serial.launch

# 启动命令速度节点
roslaunch sbus_serial sbus_cmd_vel.launch

# 启动直行控制器
roslaunch car_control control_v3.launch
```

### 串口通信协议

使用Wheeltec协议进行ROS与STM32通信：

- 波特率：115200
- 数据格式：自定义帧格式
- 包含：速度指令、转向指令、状态反馈

## 控制模式

| 模式 | 说明 | 触发方式 |
|------|------|---------|
| 手动遥控 | 遥控器直接控制 | 默认模式 |
| 自动导航 | 循迹算法控制 | 通过Web界面切换 |
| 停止模式 | 紧急停止 | 遥控器紧急按钮 |

## 相关文档

- [CAR_STARTUP_GUIDE.md](../catkin_ws/CAR_STARTUP_GUIDE.md) - 车辆启动指南
- [AVIATEUR_CAR_CONTROL_PRD.md](../AVIATEUR_CAR_CONTROL_PRD.md) - 产品需求文档
- [CAR_CONTROL_OPTIMIZATION_PRD.md](../CAR_CONTROL_OPTIMIZATION_PRD.md) - 优化需求文档

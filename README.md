# 智能车底检测与控制系统

完整的智能小车控制系统，涵盖硬件控制、ROS通信、图像处理和Web前端。

## 项目架构

```
├── car_control/              # 车辆控制模块
│   ├── stm32/                # STM32固件代码
│   ├── ros/                  # ROS节点代码
│   └── launch/               # ROS启动配置
├── ros_ws/                   # ROS工作空间
│   └── sbus_serial/          # SBUS串口通信包
├── optimize_stitching/       # 图像拼接优化模块
│   ├── tests/                # 单元测试
│   └── ...                   # 核心算法模块
├── ros_car_web_demo/         # Web前端控制界面
├── LightGlue/                # 特征匹配库
├── docs/                     # 项目文档
└── README.md                 # 项目说明
```

## 模块说明

| 模块 | 目录 | 说明 | 技术栈 |
|------|------|------|--------|
| 车辆控制 | `car_control/` | 底层控制逻辑，包含STM32固件和ROS节点 | C/C++/Python |
| ROS通信 | `ros_ws/` | SBUS串口通信包，遥控器数据接收 | C++/ROS |
| 图像拼接 | `optimize_stitching/` | 车底图像垂直拼接优化算法 | Python/OpenCV |
| Web前端 | `ros_car_web_demo/` | 可视化控制界面 | Vue.js |
| 特征匹配 | `LightGlue/` | 深度学习特征匹配库 | Python/PyTorch |

## 快速开始

### 1. 环境要求

- Python 3.8+
- OpenCV 4.x
- ROS Noetic (可选，用于车辆控制)
- Node.js 14+ (可选，用于Web前端)

### 2. 安装依赖

```bash
# 图像处理依赖
pip install opencv-python numpy pytest pytest-cov flake8

# Web前端依赖
cd ros_car_web_demo
npm install
```

### 3. 运行图像拼接

```bash
python -m optimize_stitching.main --input video.avi --output output/
```

### 4. 运行Web前端

```bash
cd ros_car_web_demo
npm run serve
```

## 功能特性

### 车辆控制
- SBUS遥控器数据接收与解析
- 串口通信协议实现
- PID控制器
- EKF状态估计
- 自动导航模式

### 图像拼接
- 垂直长图拼接（手机截长图效果）
- 多方法光流估计（ECC/Farneback/ORB）
- 自适应采样引擎
- 自然接缝检测与融合
- 图像增强（USM锐化+CLAHE）
- 降级策略管理

### Web前端
- 实时视频流显示
- 远程控制界面
- 参数配置面板
- 状态监控

## 技术文档

| 文档 | 说明 |
|------|------|
| `docs/ARCHITECTURE.md` | 系统架构设计 |
| `docs/CONTROL_SYSTEM.md` | 车辆控制系统说明 |
| `docs/IMAGE_STITCHING.md` | 图像拼接算法说明 |
| `docs/WEB_INTERFACE.md` | Web前端使用说明 |
| `docs/API_REFERENCE.md` | API接口参考 |

## 开发日志

详细开发记录请参考 `DEVELOPMENT_LOG.md`

## 许可证

MIT License

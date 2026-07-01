# 小车控制系统优化 PRD v2.0

# Car Control System Optimization - Product Requirements Document

***

## 1. 项目概述 (Project Overview)

### 1.1 背景

本项目为**货车车底智能巡检小车控制系统**，采用分层架构：

- **底层**: STM32F4 + FreeRTOS 实时控制（R550\_C30D 底盘）
- **上层**: ROS (Robot Operating System) 高级决策与路径规划

当前系统已实现基础直线巡检功能，但存在以下核心问题：

- 控制精度不足（纯航向 PID，无模型预测）
- 无轨迹规划能力（只能走固定距离直线）
- 通信协议简单（8字节原始数据，无校验/重传）
- 状态估计单一（仅编码器里程计，无融合）

### 1.2 目标

将现有系统升级为**工业级高精度巡检平台**：

- 定位精度: ±5cm → ±2cm
- 航向保持: ±3° → ±1°
- 支持复杂轨迹（曲线、多 waypoint、自动返航）
- 增强安全性与可靠性

***

## 2. 系统架构分析 (Current Architecture Analysis)

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS 上层 (Jetson Nano)                        │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │
│  │ straight_    │  │ sbus_serial  │  │ gimbal_      │             │
│  │ controller   │  │ (遥控接收)   │  │ controller   │             │
│  │ (状态机/PID) │  │              │  │ (云台控制)   │             │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘             │
│         │                 │                 │                      │
│         ▼                 ▼                 ▼                      │
│  ┌──────────────────────────────────────────────────────┐        │
│  │              ROS Topic 通信层                         │        │
│  │  /cmd_vel → /sbus ← /Odometry /ui_xy               │        │
│  └──────────────────────┬───────────────────────────────┘        │
└─────────────────────────┼─────────────────────────────────────────┘
                          │ UART/串口 (115200bps)
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    STM32 底盘 (FreeRTOS 100Hz)                      │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │
│  │ USART3 RX    │  │ Drive_Motor()│  │ Incremental_ │             │
│  │ (ROS命令解析) │  │ (运动学解算) │  │ PI_A/B/C/D   │             │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘             │
│         │                 │                 │                      │
│         ▼                 ▼                 ▼                      │
│  ┌──────────────────────────────────────────────────────┐        │
│  │              硬件抽象层 (HAL)                         │        │
│  │  Encoder(霍尔) | PWM输出 | MPU6050(IMU) | SBUS       │        │
│  └──────────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 STM32 底盘详细分析

#### 2.2.1 核心文件结构

```
R550_C30D/
├── BALANCE/
│   ├── balance.c      # 平衡控制 & 主控任务 Balance_task()
│   ├── control.c      # 运动学解算 Drive_Motor(), PID控制器
│   ├── system.c       # 系统初始化, FreeRTOS任务创建
│   └── filter.c      # 数字滤波器
├── SYSTEM/
│   ├── usart/usart.c  # 串口通信 (USART1/2/3)
│   ├── encoder/       # 霍尔编码器驱动
│   ├── imu/mpu6050.c  # IMU传感器 (MPU6050 DMP)
│   └── pwm/           # PWM电机控制
├── USER/main.c        # 主程序入口
└── BSP/               # 板级支持包
```

#### 2.2.2 关键数据结构

```c
// 电机结构体 (balance.h)
typedef struct {
    float Encoder;      // 编码器速度 (m/s)
    float Target;       // 目标速度 (m/s)
    float Motor_Pwm;    // PI输出PWM值
} Motor_t;

Motor_t MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

// 全局运动变量
float Move_X, Move_Y, Move_Z;  // 三轴目标速度 (m/s)
float RC_Velocity = 500;       // 遥控基准速度 (mm/s)

// PID参数 (Flash保存)
float Velocity_KP;    // 比例增益
float Velocity_KI;    // 积分增益
```

#### 2.2.3 控制流程 (100Hz)

```c
void Balance_task(void *pvParameters) {
    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));  // 10ms周期

        // 1. 读取编码器 → 计算实际速度
        Get_Velocity_Form_Encoder();

        // 2. 处理输入源 (APP/遥控/PS2/串口)
        if(APP_ON_Flag) Get_RC();
        else if(Remote_ON_Flag) Remote_Control();
        else if(PS2_ON_Flag) PS2_control();
        else Drive_Motor(Move_X, Move_Y, Move_Z);  // 直接使用三轴速度

        // 3. 速度环PI控制
        MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
        MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);

        // 4. PWM限幅 & 输出
        Limit_Pwm(16700);
        Set_Pwm(MOTOR_A.Motor_Pwm, ...);
    }
}
```

#### 2.2.4 当前PID实现 (增量式PI)

```c
int Incremental_PI_A(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder;
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    Pwm = constrain(Pwm, -16700, 16700);  // 限幅
    Last_bias = Bias;
    return Pwm;
}
```

**问题**: 只有P和I项，无D项；无抗积分饱和；无前馈补偿

#### 2.2.5 通信协议 (USART3 - ROS接口)

```c
// 接收缓冲区: rxbuf[8]
// 格式: [速度(8bit), 方向标志(8bit), 角度(8bit), ...]

void CAN_N_Usart_Control(void) {
    if(rxbuf[1] == 0) Move_Y = rxbuf[0];     // 正向速度
    else            Move_Y = -rxbuf[0];     // 反向速度
    Angle = (rxbuf[2] - 90) * PI / 180;     // 角度转弧度
    Kinematic_Analysis(Move_Y, Angle);       // 运动学解算
}
```

**问题**:

- 无帧头/帧尾校验
- 无CRC校验
- 无应答机制
- 数据精度低 (8bit整数)

#### 2.2.6 支持的车型模式

| 模式    | 宏定义             | 说明   | 使用的电机    |
| ----- | --------------- | ---- | -------- |
| 麦克纳姆轮 | `Mec_Car`       | 全向移动 | A,B,C,D  |
| 全向轮   | `Omni_Car`      | 3轮全向 | A,B,C    |
| 阿克曼   | `Akm_Car`       | 前轮转向 | A,B + 舵机 |
| 差速    | `Diff_Car`      | 两轮差速 | A,B      |
| 四轮    | `FourWheel_Car` | 四驱   | A,B,C,D  |
| 坦克    | `Tank_Car`      | 履带式  | A,B      |

**当前使用**: `Tank_Car` (坦克模式，两轮驱动)

### 2.3 ROS 上层详细分析

#### 2.3.1 状态机设计

```
STANDBY ──[CH6 UP]──→ ALIGN_DIRECTION ──→ AUTO_STRAIGHT ──→ WAIT_RETURN
   ↑                       │                                      │
   │                       │                              [CH5 UP] │
   │                       │                                      ↓
   └──[CH6 DOWN]──────────┴──────────────────────────── TURNING ──→ RETURN_HOME
                                                                          │
                                                                          └──→ WAIT_RETURN
```

#### 2.3.2 PID参数 (straight\_controller.cpp)

```cpp
float kp = 1.5f;          // 航向比例增益
float ki = 0.08f;         // 航向积分增益
float kd = 0.3f;          // 航向微分增益
float feedforward_compensation = 0.04f;  // 前馈补偿
float linear_x = 0.33f;   // 巡检线速度 (m/s)
float turn_speed = 0.5f;  // 调头角速度 (rad/s)
```

#### 2.3.3 遥控通道映射

| 通道         | 功能            | 阈值                  |
| ---------- | ------------- | ------------------- |
| CH1        | 左右摇杆 → 手动转向   | Deadzone=15         |
| CH2        | 前后摇杆 → 手动前进后退 | Deadzone=15         |
| CH5 (SW A) | 返航触发          | >1500               |
| CH6 (SW B) | 启动/急停         | UP>1500 / DOWN<1000 |
| CH10       | LED开关         | HIGH=1722 / LOW=282 |

***

## 3. 问题诊断与优化点 (Issues & Optimization Points)

### 3.1 🔴 严重问题 (Critical)

| #  | 问题                  | 影响          | 优先级 |
| -- | ------------------- | ----------- | --- |
| C1 | **通信协议无可靠性保障**      | 数据丢包导致小车失控  | P0  |
| C2 | **底层只有PI控制，无D项和前馈** | 响应慢，超调大     | P0  |
| C3 | **无状态观测器/卡尔曼滤波**    | 里程计漂移严重     | P0  |
| C4 | **上层PID与底层PI双环耦合**  | 参数整定困难，振荡风险 | P0  |

### 3.2 🟡 重要问题 (Major)

| #  | 问题           | 影响             | 优先级 |
| -- | ------------ | -------------- | --- |
| M1 | **无轨迹规划能力**  | 只能走固定直线，无法曲线巡检 | P1  |
| M2 | **无碰撞检测/避障** | 安全隐患           | P1  |
| M3 | **电池管理简陋**   | 仅电压阈值判断，无SOC估算 | P1  |
| M4 | **故障自恢复能力弱** | 电机堵转/失速无处理     | P1  |
| M5 | **参数固化在代码中** | 调参需重新编译烧录      | P1  |

### 3.3 🟢 改进建议 (Enhancement)

| #  | 建议                | 效益           | 优先级 |
| -- | ----------------- | ------------ | --- |
| E1 | **增加模型预测控制(MPC)** | 提升轨迹跟踪精度30%+ | P2  |
| E2 | **支持多Waypoint导航** | 扩展应用场景       | P2  |
| E3 | **Web可视化调试界面**    | 提高开发效率       | P2  |
| E4 | **日志记录与回放**       | 便于问题复现       | P2  |
| E5 | **OTA远程升级**       | 减少现场维护成本     | P3  |

***

## 4. 优化方案设计 (Optimization Solutions)

### 4.1 Phase 1: 通信协议升级 (P0)

#### 4.1.1 新协议设计: "Wheeltec Protocol V2"

```
帧格式 (20字节):
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ HEAD │ CMD  │ LEN  │ DATA (16 bytes)          │ CRC  │ TAIL │
│ 0xAA │ 1B   │ 0x10 │ ...                     │ 2B   │ 0x55 │
└──────┴──────┴──────┴─────────────────────────┴──────┴──────┘

CMD定义:
0x01: 设置速度 (Vx,Vy,Vz 各4字节float)
0x02: 设置PID参数 (KP,KI,KD 各4字节float)
0x03: 请求状态 (返回: 电压,编码器,IMU)
0x04: 急停
0x05: 复位编码器
0x06: 设置车型模式
```

#### 4.1.2 可靠性增强

- **CRC16-CCITT 校验**
- **超时重传** (50ms无响应则重发，最多3次)
- **心跳机制** (100ms一次，连续3次丢失则急停)
- **序号防重复**

### 4.2 Phase 2: 底层控制算法升级 (P0-P1)

#### 4.2.1 PID → PID++ 升级

```c
// 新增: 完整PID + 抗积分饱和 + 变死区 + 变化率限制
typedef struct {
    float Kp, Ki, Kd;
    float Kf;           // 前馈增益
    float integral;     // 积分累积
    float integral_max; // 积分限幅
    float prev_error;   // 上次误差
    float prev_output;  // 上次输出
    float output_rate_limit; // 输出变化率限制
    float deadzone;     // 死区
} PID_Controller_t;

float PID_Update(PID_Controller_t* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // 死区处理
    if(fabs(error) < pid->deadzone) error = 0;
    
    // 抗积分饱和
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max);
    
    // 计算各项
    float P = pid->Kp * error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (error - pid->prev_error) / dt;
    float F = pid->Kf * setpoint;  // 前馈
    
    float output = P + I + D + F;
    
    // 变化率限制
    output = constrain(output,
                      pid->prev_output - pid->output_rate_limit,
                      pid->prev_output + pid->output_rate_limit);
    
    pid->prev_error = error;
    pid->prev_output = output;
    return output;
}
```

#### 4.2.2 卡尔曼滤波状态估计

```c
// 状态向量: [位置X, 位置Y, 航向θ, 速度Vx, 速度Vy, 角速度ω]
// 观测量: 编码器(左右轮速), IMU(加速度, 角速度)

#define STATE_DIM 6
#define MEAS_DIM 4

// 状态转移矩阵 F (恒速模型)
// 观测矩阵 H
// 过程噪声 Q
// 观测噪声 R

void Kalman_Predict(Kalman_t* k, float dt);
void Kalman_Update(Kalman_t* k, float* z);
```

#### 4.2.3 自适应PID (可选)

```c
// 基于误差大小的自适应增益调整
void Adaptive_PID_Tune(PID_Controller_t* pid, float error) {
    float error_mag = fabs(error);
    if(error_mag > 1.0) {
        // 大误差: 增大Kp加快响应
        pid->Kp_active = pid->Kp * 1.5;
    } else if(error_mag < 0.1) {
        // 小误差: 增大Ki消除稳态误差
        pid->Ki_active = pid->Ki * 2.0;
    } else {
        pid->Kp_active = pid->Kp;
        pid->Ki_active = pid->Ki;
    }
}
```

### 4.3 Phase 3: 上层规划与决策升级 (P1-P2)

#### 4.3.1 多Waypoint导航

```cpp
// 新增: WaypointManager 类
class WaypointManager {
public:
    struct Waypoint {
        float x, y;          // 目标位置
        float yaw;           // 目标航向
        float tolerance;     // 到达阈值
        float max_speed;     // 最大速度
        bool look_ahead;     // 是否提前转向
    };
    
    void addWaypoint(float x, float y, float yaw = NAN);
    void clearWaypoints();
    bool isComplete();
    geometry_msgs::Twist computeCommand(const nav_msgs::Odometry& odom);
    
private:
    std::vector<Waypoint> waypoints_;
    int current_index_;
    PurePursuitController pure_pursuit_;  // 纯追踪算法
};
```

#### 4.3.2 纯追踪算法 (Pure Pursuit)

```cpp
class PurePursuitController {
public:
    geometry_msgs::Twist compute(
        float robot_x, float robot_y, float robot_yaw,
        float target_x, float target_y,
        float lookahead_dist  // 前视距离
    ) {
        // 计算局部坐标系下的目标点
        float dx = target_x - robot_x;
        float dy = target_y - robot_y;
        float local_x = dx * cos(-robot_yaw) - dy * sin(-robot_yaw);
        float local_y = dx * sin(-robot_yaw) + dy * cos(-robot_yaw);
        
        // 计算曲率
        float curvature = 2.0 * local_y / (local_x*local_x + local_y*local_y);
        
        // 限制曲率
        curvature = constrain(curvature, -max_curvature_, max_curvature_);
        
        // 输出线速度和角速度
        geometry_msgs::Twist cmd;
        cmd.linear.x = base_speed_;
        cmd.angular.z = cmd.linear.x * curvature;
        return cmd;
    }
private:
    float base_speed_ = 0.3f;
    float max_curvature_ = 2.0f;
};
```

#### 4.3.3 MPC模型预测控制 (高级选项)

```
// 使用 acados 或 osqp 求解器
// 状态: [x, y, θ, v, ω]
// 控制: [a, α] (线加速度, 角加速度)
// 约束:
//   - 速度范围: [-1.0, 1.0] m/s
//   - 角速度范围: [-π/2, π/2] rad/s
//   - 加速度范围: [-2.0, 2.0] m/s²
//   - 避障约束 (动态障碍物)
```

### 4.4 Phase 4: 安全与可靠性 (P1)

#### 4.41 看门狗系统

```cpp
class SafetyWatchdog {
    enum class Level { WARNING, CRITICAL, EMERGENCY };
    
    void check() {
        // 1. 通信看门狗 (>200ms无数据 → WARNING, >500ms → EMERGENCY)
        // 2. 电压监控 (<10.8V → WARNING, <10.0V → CRITICAL)
        // 3. 电机电流监控 (过流保护)
        // 4. IMU倾角监控 (>30° → EMERGENCY)
        // 5. 速度异常检测 (期望vs实际偏差过大)
    }
};
```

#### 4.4.2 故障诊断与恢复

```c
enum FaultCode {
    FAULT_NONE = 0,
    FAULT_ENCODER_LOST,      // 编码器信号丢失
    FAULT_MOTOR_STALL,       // 电机堵转
    FAULT_IMU_ERROR,         // IMU通信失败
    FAULT_LOW_VOLTAGE,       // 低电压
    FAULT_COMM_TIMEOUT,      // 通信超时
    FAULT_OVER_CURRENT,      // 过流
    FAULT_OVER_TEMP          // 过温
};

void Fault_Handler(FaultCode fault) {
    switch(fault) {
        case FAULT_MOTOR_STALL:
            // 尝试反向转动脱困
            Drive_Motor(-0.1, 0, 0);
            vTaskDelay(500);
            break;
        case FAULT_ENCODER_LOST:
            // 切换到开环速度控制
            open_loop_mode = true;
            break;
        // ...
    }
}
```

### 4.5 Phase 5: Web可视化平台 (P2)

基于之前的 `WEB_CONTROL_PRD.md` 设计，集成以下功能：

| 模块   | 功能              |
| ---- | --------------- |
| 实时监控 | 位置、速度、电量、温度曲线图  |
| 参数调节 | 在线修改PID参数并下发到底层 |
| 轨迹编辑 | 拖拽设置Waypoint    |
| 日志查看 | 下载运行日志用于离线分析    |
| 固件升级 | OTA更新STM32固件    |

***

## 5. 开发路线图 (Development Roadmap)

### Milestone 1: 通信协议重构 (Week 1-2)

- [ ] 设计 Wheeltec Protocol V2 规范文档
- [ ] STM32端: USART3 接收改为中断+FIFO
- [ ] STM32端: 实现 CRC16 + 帧解析
- [ ] ROS端: 实现 serial\_node.py (Python/C++)
- [ ] 测试: 误码率测试、压力测试

### Milestone 2: 底层PID升级 (Week 3-4)

- [ ] 重写 PID 控制器 (完整版)
- [ ] 添加前馈补偿 (基于目标速度)
- [ ] 添加抗积分饱和
- [ ] Flash参数表扩展 (新增KD, KF等)
- [ ] 台架测试: 阶跃响应、正弦跟踪

### Milestone 3: 状态估计 (Week 5-6)

- [ ] 实现扩展卡尔曼滤波 (EKF)
- [ ] 融合编码器 + IMU 数据
- [ ] 发布 /odom\_imu\_fused topic
- [ ] 对比测试: 有/无EKF的定位精度

### Milestone 4: 导航升级 (Week 7-9)

- [ ] 实现 WaypointManager
- [ ] 实现纯追踪控制器
- [ ] 修改 straight\_controller 状态机
- [ ] 实地测试: 曲线轨迹跟踪

### Milestone 5: 安全增强 (Week 10-11)

- [ ] 实现看门狗系统
- [ ] 实现故障诊断与恢复
- [ ] 电池SOC估算库伦计法
- [ ] 应急测试: 通信断开、低电压场景

### Milestone 6: Web平台 (Week 12-14)

- [ ] 后端 API 开发
- [ ] 前端界面开发
- [ ] 与 ROS 桥接
- [ ] 集成测试

***

## 6. 技术选型建议 (Technology Recommendations)

| 组件    | 当前方案       | 建议方案             | 理由        |
| ----- | ---------- | ---------------- | --------- |
| 底盘OS  | FreeRTOS   | FreeRTOS (保留)    | 成熟稳定，实时性好 |
| 通信协议  | 8字节裸数据     | COBS/NMEA风格      | 可靠性提升     |
| 状态估计  | 仅编码器       | EKF (编码器+IMU)    | 精度提升50%+  |
| 路径跟踪  | PID航向保持    | 纯追踪/MPC          | 支持曲线      |
| 上层框架  | ROS Noetic | 保留或迁移ROS2        | 生态兼容      |
| Web前端 | -          | Vue3 + ECharts   | 轻量高效      |
| 参数存储  | 内部Flash    | Flash分区+ROS参数服务器 | 灵活调参      |

***

## 7. 性能指标预期 (Expected Performance Metrics)

| 指标     | 当前值   | 目标值    | 提升   |
| ------ | ----- | ------ | ---- |
| 直线跟踪误差 | ±15cm | ±3cm   | 80%↓ |
| 航向保持精度 | ±5°   | ±1°    | 80%↓ |
| 到达精度   | ±20cm | ±5cm   | 75%↓ |
| 通信丢包率  | \~1%  | <0.01% | 99%↓ |
| 启动时间   | \~60s | <15s   | 75%↓ |
| 故障恢复时间 | 人工介入  | 自动<2s  | -    |

***

## 8. 风险评估 (Risk Assessment)

| 风险              | 概率 | 影响 | 缓解措施                    |
| --------------- | -- | -- | ----------------------- |
| STM32 Flash空间不足 | 中  | 高  | 优化代码，移除未用功能             |
| 通信延迟增大          | 低  | 中  | 保持115200波特率，减少数据量       |
| PID参数整定困难       | 中  | 中  | 使用Ziegler-Nichols法+自动整定 |
| EKF发散           | 低  | 高  | 充分的初始对准+协方差监控           |
| ROS节点崩溃         | 低  | 中  | 进程守护(systemd/watchdog)  |

***

## 9. 附录 (Appendix)

### A. 关键文件索引

```
STM32底层:
- R550_C30D/BALANCE/balance.c      # 主控任务 (L296-L602)
- R550_C30D/BALANCE/control.c      # PID控制器 (L177-L825)
- R550_C30D/SYSTEM/usart/usart.c   # 串口驱动
- R550_C30D/USER/main.c            # 程序入口

ROS上层:
- catkin_ws/src/turn_on_wheeltec_robot/src/straight_controller.cpp  # 状态机控制
- catkin_ws/car_autostart.sh        # 启动脚本
```

### B. 通信时序图

```
ROS (50Hz)              STM32 (100Hz)
   │                        │
   ├─ /cmd_vel ────────────►│ USART3 RX ISR
   │  {Vx,Vy,Vz}            │ 解析→Move_X,Y,Z
   │                        │
   │◄── /Odometry ──────────┤ Balance_task()
   │  {pos,yaw,speed}       │ 读取编码器
   │                        │ PI计算→PWM输出
   │                        │
   ├─ Heartbeat ───────────►│ 心跳应答
   │                        │
```

### C. PID参数参考值

```yaml
# velocity_loop (底层速度环)
Velocity_KP: 300.0    # 比例增益
Velocity_KI: 50.0     # 积分增益  
Velocity_KD: 20.0     # 微分增益 (新增)
Velocity_KF: 100.0    # 前馈增益 (新增)
Integral_Max: 5000.0   # 积分限幅
Output_Rate_Limit: 5000.0  # 输出变化率限制
Deadzone: 0.02         # 速度死区 (m/s)

# heading_hold (上层航向环)
Heading_KP: 1.5
Heading_KI: 0.08
Heading_KD: 0.3
Feedforward: 0.04
Max_Angular: 0.5 rad/s
```

***

**文档版本**: v2.0
**创建日期**: 2026-04-27
**作者**: AI Assistant
**状态**: 待评审

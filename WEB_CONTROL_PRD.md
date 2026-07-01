# 车底巡检小车 Web 可视化控制平台 PRD v1.0

## 1. 项目概述

### 1.1 背景
- **现有系统**：两轮差速履带式车底巡检小车，搭载广角/鱼眼摄像头朝下拍摄
- **后端已就绪**：
  - `straight_controller.cpp` v2.0 — 三模式状态机（STANDBY/AUTO/MANUAL）
  - `car_autostart.sh` v2.0 — 加速启动脚本
  - `undercarriage_pipeline.py` — 视频→去畸变→拼接全景图流水线
  - `calibrate_camera.py` — 相机标定工具（输出 `.yml` 参数文件）
  - SBUS 遥控接收节点 (`sbus_cmd_vel_node.cpp`) — 已发布 `/sbus` 话题
- **前端已有**：`ros_car_web_demo` (Vue3 + Element Plus + roslib + video.js)
- **目标**：将现有 Web Demo 改造为**车底巡检专用可视化控制平台**

### 1.2 核心需求
| # | 需求 | 优先级 | 状态 |
|---|------|--------|------|
| R1 | 实时视频流显示（车底画面） | P0 | Demo已有基础 |
| R2 | 运动控制（前进/后退/转向/停止） | P0 | Demo已有基础 |
| R3 | 模式切换可视化（STANDBY↔AUTO↔MANUAL） | P0 | **需新增** |
| R4 | 远程启动/急停按钮（替代CH6遥控） | P0 | **需新增** |
| R5 | 拼接全景图实时展示 | P1 | Demo已有框架 |
| R6 | 小车状态监控（里程计/电池/模式） | P1 | Demo有部分 |
| R7 | 云台控制（锁定90°朝上） | P1 | **需新增** |
| R8 | 拼接参数配置（帧间隔/最大帧数/标定文件） | P2 | **需新增** |
| R9 | 历史任务记录与回放 | P2 | **需新增** |

---

## 2. 现有 Web Demo 分析

### 2.1 技术栈
```
Vue 3 + Composition API (setup语法)
Element Plus UI组件库
roslib.js (WebSocket → ROSBridge)
video.js (视频播放)
axios (HTTP API调用)
vue-cli 5 (构建工具)
```

### 2.2 现有功能模块

| 模块 | 文件 | 功能 | 适配状态 |
|------|------|------|---------|
| ROS连接 | [App.vue:113-168](file:///c:\Users\admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L113-L168) | WebSocket连接ROSBridge | ✅ 可复用，需改默认URL |
| 运动控制 | [App.vue:215-226](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L215-L226) | 发布/cmd_vel Twist消息 | ✅ 可复用，需增强 |
| 视频播放 | [App.vue:76-83](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L76-L83) | video.js播放MJPEG/FLV | ⚠️ 需适配车底摄像头topic |
| 传感器面板 | [App.vue:59-66](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L59-L66) | 显示速度/电池/障碍物 | ⚠️ 需适配实际话题 |
| 拼接图展示 | [App.vue:87-99](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L87-L99) | 显示Base64拼接图像 | ⚠️ 需适配新话题 |

### 2.3 现有不足（需改造）
1. ❌ **无模式状态显示** — 不知道当前是 STANDBY/AUTO/MANUAL
2. ❌ **无远程启动/急停** — 只能通过物理遥控器操作
3. ❌ **无云台控制** — 无法远程调整云台角度
4. ❌ **无拼接触发** — 无法从Web端发起拼接任务
5. ❌ **无SBUS数据可视化** — 看不到遥控器通道值
6. ❌ **硬编码IP地址** — 不方便切换不同小车

---

## 3. 系统架构设计

### 3.1 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    浏览器 (Web Frontend)                      │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────────┐ │
│  │ 视频监控  │ │ 运动控制  │ │ 模式管理  │ │  拼接结果查看   │ │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └───────┬────────┘ │
│       │            │           │                │          │
│  ┌────▼────────────▼───────────▼────────────────▼────────┐ │
│  │              roslib.js (WebSocket)                     │ │
│  └──────────────────────┬────────────────────────────────┘ │
└─────────────────────────┼───────────────────────────────────┘
                          │ ws://<小车IP>:9090
                    ┌─────▼──────────┐
                    │   ROSBridge    │
                    │  (rosbridge_server)
                    └─────┬──────────┘
              ┌────────────┼────────────┐
              │            │            │
     ┌────────▼──┐  ┌──────▼─────┐ ┌───▼──────────┐
     │ /cmd_vel  │  │ /sbus      │ │ /ui_xy       │
     │ (运动指令) │  │ (遥控数据)  │ │ (云台控制)    │
     └─────┬──────┘  └──────┬─────┘ └──────┬───────┘
           │                 │               │
     ┌─────▼─────────────────▼───────────────▼───────┐
     │         straight_controller.cpp v2.0          │
     │  STANDBY ←→ AUTO ←→ MANUAL                   │
     │  (CH6遥控 / Web按钮 双通道控制)                 │
     └──────────────────────────────────────────────┘
```

### 3.2 ROS Topic 映射表

| 方向 | Topic名 | 消息类型 | 用途 | 来源/去向 |
|------|---------|----------|------|-----------|
| Pub | `/cmd_vel` | geometry_msgs/Twist | 运动指令 | Web → 控制器 |
| Pub | `/ui_xy` | std_msgs/String | 云台角度 "0,90" | Web → 云台 |
| Sub | `/sbus` | sbus_serial/Sbus | 遥控器原始数据 | SBUS节点 → Web |
| Sub | `/Odometry` | nav_msgs/Odometry | 里程计数据 | 底盘驱动 → Web |
| Sub | `/camera/image_raw` | sensor_msgs/Image | 车底原始画面 | 摄像头 → Web |
| Sub | `/car_bottom_stitch` | sensor_msgs/Image | 拼接全景图 | 拼接服务 → Web |
| Service | `/trigger_stitch` | std_srvs/Trigger | 触发拼接任务 | Web → 后端 |
| Service | `/set_mode` | car_pkg/SetMode | 切换运行模式 | Web → 控制器 |

### 3.3 页面布局设计

```
┌──────────────────────────────────────────────────────────────────┐
│  🚗 车底巡检控制台 v1.0        [STANDBY] ● 连接中    🔧 设置 ⚙️  │
├────────────┬─────────────────────────────────────────────────────┤
│            │                                                     │
│  📡 连接    │  ┌──────────────────┐  ┌────────────────────────┐  │
│  IP:______  │  │                  │  │                        │  │
│  [连接]     │  │   车底实时视频     │  │   拼接全景图            │  │
│            │  │   (video.js)      │  │   (可缩放/滚动)         │  │
│  🎮 控制    │  │                  │  │                        │  │
│  [▶启动]    │  │                  │  │                        │  │
│  [⏹急停]   │  │                  │  │                        │  │
│  [🔄返航]   │  │                  │  │                        │  │
│            │  └──────────────────┘  └────────────────────────┘  │
│  📊 状态    │                                                     │
│  模式:STANDY│  ┌──────────────────────────────────────────────┐  │
│  X: 0.00m   │  │  🕹️ 虚拟摇杆 / 键盘控制区                    │  │
│  Y: 0.00m   │  │       [W]                                     │  │
│ Yaw: 0.0°  │  │    [A][S][D]    速度: ████░░░░ 0.33 m/s       │  │
│  电池:12.5V │  │                                              │  │
│  帧率:30fps │  └──────────────────────────────────────────────┘  │
│            │                                                     │
│  📷 云台    │  ┌──────────────────────────────────────────────┐  │
│  俯仰:90°  │  │  📈 任务进度条                                 │  │
│  [锁定上]   │  │  ████████████░░░░░░░░  65%  剩余5.6m           │  │
│            │  └──────────────────────────────────────────────┘  │
│  📋 日志    │                                                     │
│  [INFO]...  │                                                     │
│            │                                                     │
└────────────┴─────────────────────────────────────────────────────┘
```

---

## 4. 详细功能规格

### 4.1 R1: 实时视频流显示

**现状**：[App.vue:76-83](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L76-L83) 使用 video.js + FLV/MJPEG

**改造方案**：

```javascript
// 方案A：web_video_server + MJPEG（推荐，最简单）
const videoStreamUrl = ref('http://{IP}:8080/stream?topic=/camera/image_raw')

// 方案B：ROSBridge Image topic + Canvas（延迟较高但无需额外服务）
// 订阅 /camera/image_raw → Base64 → <img> 显示

// 方案C：WebRTC（最低延迟，需额外组件）
```

**决策**：方案A为主，方案B为fallback。需在小车上部署 `web_video_server`

**UI改进**：
- 全屏按钮
- 截图保存功能
- 分辨率/帧率显示叠加层

### 4.2 R2: 运动控制增强

**现状**：[App.vue:36-56](file:///c:\Users/admin\PycharmProjects\pythonProject/ros_car_web_demo/src/App.vue#L36-L56) 固定速度按钮

**改造方案**：

| 控件类型 | 说明 |
|---------|------|
| **虚拟摇杆** (nipplejs) | 手机端触控支持 |
| **键盘映射** | WASD/方向键 |
| **速度滑块** | 0~0.5 m/s 可调 |
| **微动按钮** | 精细调整 ±0.05 m/s |

```javascript
// 新增：持续发送机制（按下不松开时持续发布）
let cmdInterval = null
const startContinuousCmd = (lx, az) => {
  sendCmd(lx, az)
  cmdInterval = setInterval(() => sendCmd(lx, az), 100) // 10Hz
}
const stopContinuousCmd = () => {
  if (cmdInterval) clearInterval(cmdInterval)
  cmdInterval = null
  sendCmd(0, 0) // 松开即停
}
```

### 4.3 R3: 模式切换可视化（核心新增）

**对应后端**：`straight_controller.cpp` 的三模式状态机

```javascript
// 新增状态管理
const opMode = ref('STANDBY')  // STANDBY | AUTO | MANUAL
const ctrlState = ref('IDLE')  // IDLE | ALIGN | DRIVE | WAIT_RET | RETURN | TURNING
const missionProgress = ref(0) // 0-100%

// 订阅控制器状态话题（需在C++端新增发布）
const modeSub = new ROSLIB.Topic({
  ros: ros,
  name: '/controller_state',
  messageType: 'car_pkg/ControllerState'  // 自定义消息
})
modeSub.subscribe((msg) => {
  opMode.value = msg.mode      // string: STANDBY/AUTO/MANUAL
  ctrlState.value = msg.state   // string: 当前子状态
  missionProgress.value = msg.progress  // float32: 0-100
})
```

**UI设计**：

```
┌─────────────────────────────────────┐
│  运行模式                            │
│  ┌─────────┐ ┌─────────┐ ┌────────┐ │
│  │ STANDBY │ │  AUTO   │ │MANUAL │ │
│  │  ○ 待机  │ │  ○ 自动 │ │○ 手动 │ │
│  └─────────┘ └─────────┘ └────────┘ │
│                                      │
│  当前状态: [ALIGN_DIRECTION] 对准方向  │
│  进度: ████████████░░░░ 65%          │
│  剩余距离: 5.6m                       │
└─────────────────────────────────────┘
```

### 4.4 R4: 远程启动/急停（核心新增）

**对应后端**：替代 CH6 (SW B) 物理遥控操作

```html
<!-- 新增控制按钮组 -->
<div class="control-buttons">
  <!-- 启动按钮（仅STANDBY模式下可用）-->
  <el-button type="success" size="large"
             :disabled="opMode !== 'STANDBY'"
             @click="startAutoMission">
    ▶ 启动巡检
  </el-button>

  <!-- 急停按钮（任何非STANDBY模式下可用）-->
  <el-button type="danger" size="large"
             :disabled="opMode === 'STANDBY'"
             @click="emergencyStop"
             class="e-stop-btn">
    ⏹ 急停
  </el-button>

  <!-- 返航按钮（AUTO模式下到达目标后可用）-->
  <el-button type="warning" size="large"
             :disabled="ctrlState !== 'WAIT_RETURN'"
             @click="triggerReturn">
    🔄 返航
  </el-button>
</div>
```

```javascript
// 通过Service或Topic发送模式切换指令
const startAutoMission = () => {
  // 方式A：调用ROS Service
  const srv = new ROSLIB.Service({
    ros: ros,
    name: '/set_mode',
    serviceType: 'car_pkg/SetMode'
  })
  const request = new ROSLIB.ServiceRequest({ mode: 'AUTO' })
  srv.callService(request, (result) => {
    ElMessage.success(result.success ? '巡检已启动!' : '启动失败')
  })

  // 方式B（备选）：直接发布到Topic
  const modePub = new ROSLIB.Topic({
    ros: ros,
    name: '/mode_command',
    messageType: 'std_msgs/String'
  })
  modePub.publish(new ROSLIB.Message({ data: 'START_AUTO' }))
}

const emergencyStop = () => {
  ElMessageBox.confirm('确认紧急停车?', '⚠️ 急停确认', {
    confirmButtonText: '立即停车',
    cancelButtonText: '取消',
    type: 'warning'
  }).then(() => {
    modePub.publish(new ROSLIB.Message({ data: 'EMERGENCY_STOP' }))
    ElMessage.error('已发送急停指令!')
  }).catch(() => {})
}
```

**安全设计**：
- 急停按钮需要二次确认弹窗
- 急停按钮红色醒目 + 脉冲动画
- STANDBY 模式下禁用急停（已经在待机了）
- 同时保留物理遥控器 CH6 作为冗余急停通道

### 4.5 R7: 云台控制

```html
<!-- 云台控制面板 -->
<el-card title="云台控制">
  <el-row gutter="10">
    <el-col :span="12">
      <span>俯仰角: {{ gimbalPitch }}°</span>
      <el-slider v-model="gimbalPitch" :min="-45" :max="135"
                 @change="sendGimbalCmd"></el-slider>
    </el-col>
    <el-col :span="12">
      <el-button type="primary" @click="lockGimbalUp">锁定朝上(90°)</el-button>
      <el-button @click="lockGimbalDown">锁定朝下</el-button>
    </el-col>
  </el-row>
</el-card>

<script setup>
const gimbalPitch = ref(90)
const gimbalCmdPub = null

const initGimbalTopic = () => {
  gimbalCmdPub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui_xy',
    messageType: 'std_msgs/String'
  })
}

const sendGimbalCmd = (val) => {
  if (!gimbalCmdPub) return
  // 格式: "yaw,pitch" (单位:度)
  gimbalCmdPub.publish(new ROSLIB.Message({ data: `0,${val}` }))
}

const lockGimbalUp = () => { gimbalPitch.value = 90; sendGimbalCmd(90) }
const lockGimbalDown = () => { gimbalPitch.value = -45; sendGimbalCmd(-45) }
</script>
```

### 4.6 R8: 拼接参数配置 & 触发

```html
<!-- 拼接任务配置 -->
<el-card title="拼接设置">
  <el-form label-width="100px">
    <el-form-item label="标定文件">
      <el-select v-model="stitchConfig.calibFile">
        <option value="tank_camera_params.yml">tank_camera_params.yml</option>
        <option value="wide_camera_params.yml">wide_camera_params.yml</option>
      </el-select>
    </el-form-item>
    <el-form-item label="截帧间隔">
      <el-input-number v-model="stitchConfig.interval" :min="1" :max="10" />
    </el-form-item>
    <el-form-item label="最大帧数">
      <el-input-number v-model="stitchConfig.maxFrames" :min="10" :max="500" />
    </el-form-item>
    <el-form-item>
      <el-button type="primary" @click="triggerStitching">开始拼接</el-button>
    </el-form-item>
  </el-form>
</el-card>

<script setup>
const stitchConfig = ref({
  calibFile: 'tank_camera_params.yml',
  interval: 5,
  maxFrames: 200,
  videoPath: ''
})

const triggerStitching = () => {
  // 通过HTTP API调用后端Python拼接服务
  axios.post('/api/stitch', stitchConfig.value).then(res => {
    ElMessage.success(`拼接任务已提交! Task ID: ${res.data.task_id}`)
  })
}
```

---

## 5. 后端 C++ 改造需求

### 5.1 新增 ControllerState 消息定义

```bash
# 在 catkin_ws/src 下新建 car_msgs 包
catkin_create_pkg car_msgs message_generation std_msgs

# 定义 car_msgs/msg/ControllerState.msg
string mode           # STANDBY / AUTO / MANUAL
string state          # INIT / ALIGN_DIRECTION / AUTO_STRAIGHT / WAIT_RETURN / RETURN_HOME / TURNING
float32 progress      # 0.0 ~ 100.0
float32 remaining_dist # meters to target
float32 current_x
float32 current_y
float32 current_yaw
string status_msg     # human-readable status
```

### 5.2 straight_controller.cpp 新增发布逻辑

在 `sbusCallback()` 和 `odomCallback()` 中添加状态发布：

```cpp
// 新增 Publisher
ros::Publisher state_pub;

// 在 main() 中初始化
state_pub = nh.advertise<car_msgs::ControllerState>("/controller_state", 10);

// 在每次状态变化时发布
void publishControllerState() {
    car_msgs::ControllerState state_msg;
    switch(op_mode) {
        case OperationMode::STANDBY_MODE: state_msg.mode = "STANDBY"; break;
        case OperationMode::AUTO_MODE:    state_msg.mode = "AUTO"; break;
        case OperationMode::MANUAL_MODE:  state_msg.mode = "MANUAL"; break;
    }
    switch(current_state) {
        case ControlState::STANDBY:          state_msg.state = "STANDBY"; break;
        case ControlState::INIT:             state_msg.state = "INIT"; break;
        case ControlState::ALIGN_DIRECTION:  state_msg.state = "ALIGN_DIRECTION"; break;
        case ControlState::AUTO_STRAIGHT:    state_msg.state = "AUTO_STRAIGHT"; break;
        case ControlState::WAIT_RETURN:     state_msg.state = "WAIT_RETURN"; break;
        case ControlState::RETURN_HOME:      state_msg.state = "RETURN_HOME"; break;
        case ControlState::TURNING:          state_msg.state = "TURNING"; break;
    }
    state_msg.remaining_dist = calculateDistance(...);
    state_msg.current_x = current_odom.pose.pose.position.x;
    state_msg.current_y = current_odom.pose.pose.position.y;
    state_pub.publish(state_msg);
}
```

### 5.3 新增 SetMode Service 定义

```yaml
# car_msgs/srv/SetMode.srv
string requested_mode  # "START_AUTO" | "EMERGENCY_STOP" | "TRIGGER_RETURN"
---
bool success
string message
```

---

## 6. 组件拆分计划

将单文件 `App.vue` 拆分为可维护的组件结构：

```
src/
├── App.vue                    # 主布局容器
├── main.js                   # 入口
├── components/
│   ├── RosConnection.vue     # ROS连接管理 (R1基础)
│   ├── VideoMonitor.vue      # 视频流显示 (R1)
│   ├── MotionControl.vue     # 运动控制+虚拟摇杆 (R2)
│   ├── ModePanel.vue         # 模式切换+状态显示 (R3) ★新增
│   ├── SafetyControl.vue     # 启动/急停/返航按钮 (R4) ★新增
│   ├── GimbalControl.vue     # 云台控制 (R7) ★新增
│   ├── SensorDashboard.vue   # 传感器仪表盘 (R6)
│   ├── StitchViewer.vue      # 拼接图查看器 (R5)
│   └── StitchConfig.vue      # 拼接参数配置 (R8) ★新增
├── composables/
│   ├── useRosConnection.js   # ROS连接Hook
│   ├── useMotionControl.js   # 运动控制Hook
│   └── useControllerState.js # 控制器状态Hook ★新增
├── utils/
│   ├── rosTopics.js          # Topic常量定义
│   └── config.js             # 默认配置(IP/端口等)
└── assets/
    └── logo.png
```

---

## 7. 开发里程碑

| Phase | 内容 | 交付物 | 预估工作量 |
|-------|------|--------|-----------|
| **P0-M1** | 基础适配：修复连接+视频+运动控制 | 可运行的Web控制页面 | 0.5天 |
| **P0-M2** | 核心功能：模式显示+远程启停+云台控制 | 完整控制能力 | 1天 |
| **P0-M3** | 后端对接：ControllerState消息+SetMode Service | Web ↔ 控制器打通 | 1天 |
| **P1-M4** | 拼接集成：参数配置+触发+结果展示 | 端到端拼接流程 | 1天 |
| **P2-M5** | 体验优化：虚拟摇杆+键盘+移动端适配 | 生产级可用 | 1天 |

---

## 8. 关键技术风险

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| ROSBridge WebSocket延迟 | 控制响应慢 | 本地局域网部署(<10ms)；关键操作用Service确认 |
| video.js 兼容性 | 部分浏览器不支持FLV | 备选方案B(Canvas渲染) |
| C++端新增消息编译失败 | 后端无法启动 | 先本地测试msg/srv编译 |
| 多客户端并发控制 | 两个浏览器同时操作冲突 | 服务端互斥锁；后接入先得 |
| 急停网络延迟 | 急停指令不能即时到达 | 物理CH6作为最终兜底；Web急停加确认 |

---

## 9. 附录

### A. ROSBridge 安装（小车端）
```bash
sudo apt install ros-noetic-rosbridge-server
# 或
roslaunch rosbridge_server rosbridge_websocket.launch
# 默认监听 ws://0.0.0.0:9090
```

### B. web_video_server 安装（视频流）
```bash
cd ~/catkin_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
catkin_make
roslaunch web_video_server web_video_server.launch
# 默认 http://0.0.0.0:8080
```

### C. 默认网络配置
```
小车IP: 192.168.1.100 (静态IP)
ROSBridge: ws://192.168.1.100:9090
视频流:   http://192.168.1.100:8080/stream?topic=/camera/image_raw
拼接API: http://192.168.1.100:5000/api/stitch  (Flask/FastAPI)
```

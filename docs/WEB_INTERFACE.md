# Web 前端界面

## 项目结构

```
ros_car_web_demo/
├── public/                   # 静态资源
│   ├── favicon.ico           # 网站图标
│   └── index.html            # HTML模板
├── src/                      # 源代码
│   ├── assets/               # 资源文件
│   │   └── logo.png          # 应用Logo
│   ├── App.vue               # 主应用组件
│   └── main.js               # 入口文件
├── .gitignore                # Git忽略配置
├── README.md                 # 项目说明
├── babel.config.js           # Babel配置
├── jsconfig.json             # JavaScript配置
├── package-lock.json         # 依赖锁定
├── package.json              # 依赖配置
└── vue.config.js             # Vue配置
```

## 功能说明

### 1. 实时视频流显示

- 显示摄像头实时画面
- 支持视频分辨率调整
- 全屏显示模式

### 2. 远程控制界面

- 虚拟摇杆控制
- 速度和转向滑块
- 控制模式切换（手动/自动）

### 3. 参数配置面板

- 调整控制参数
- 保存配置到本地存储
- 预设配置快速切换

### 4. 状态监控

- 车辆状态实时显示
- 电池电量监控
- 错误日志显示

## 使用方法

### 安装依赖

```bash
cd ros_car_web_demo
npm install
```

### 开发模式

```bash
npm run serve
```

### 生产构建

```bash
npm run build
```

### 部署

```bash
# 将 dist/ 目录部署到 Web 服务器
```

## 技术栈

| 技术 | 版本 | 说明 |
|------|------|------|
| Vue.js | 3.x | 前端框架 |
| Vue CLI | 4.x | 构建工具 |
| Bootstrap | 4.x | UI样式 |
| WebSocket | - | 实时通信 |

## 相关文档

- [WEB_CONTROL_PRD.md](../WEB_CONTROL_PRD.md) - Web控制产品需求文档

# Aviateur + Car Control - Windows 源码编译完整指南

## 📋 前置要求

| 软件 | 版本 | 用途 | 下载链接 |
|------|------|------|---------|
| **Git** | 2.x | 版本控制 | https://git-scm.com/download/win |
| **CMake** | 3.20+ | 构建系统 | https://cmake.org/download/ |
| **Visual Studio 2022** | Community/Build Tools | C++编译器 | https://visualstudio.microsoft.com/downloads/ |
| **vcpkg** | 最新 | C++包管理器 | 自动克隆安装 |

**硬件要求：**
- 磁盘空间: 至少10GB（依赖库较大）
- 内存: 建议8GB+
- 网络: 首次需下载约2GB依赖

---

## 🔧 方式一：全自动脚本（推荐新手）

### 步骤1：下载脚本

将 `setup_and_build.bat` 复制到一个方便的位置（如桌面）

```bash
# 从项目目录复制
copy c:\Users\admin\PycharmProjects\pythonProject\aviateur_car_control\setup_and_build.bat %USERPROFILE%\Desktop\
```

### 步骤2：以管理员身份运行

```
1. 右键点击 setup_and_build.bat
2. 选择 "以管理员身份运行"
3. 等待自动完成（首次约30-60分钟）
```

**脚本会自动完成：**
- ✅ 检测/安装 Git, CMake
- ✅ 检测/安装 Visual Studio Build Tools
- ✅ 克隆并配置 vcpkg
- ✅ 安装所有依赖库 (libusb, ffmpeg, opencv等)
- ✅ 克隆 Aviateur 源码
- ✅ 集成 Car Control 模块
- ✅ 编译生成 exe

### 步骤3：使用编译结果

编译成功后：
- 可执行文件: `C:\dev\aviateur_build\aviateur\build\bin\Release\aviateur.exe`
- 桌面快捷方式: `Aviateer_CarControl.exe` (如成功创建)

---

## 🔧 方式二：手动逐步操作（推荐进阶用户）

如果自动脚本遇到问题，或你想了解每一步的细节：

### 第1步：安装 Visual Studio 2022

**选项A: 完整版 (Community，免费)**
```
1. 访问 https://visualstudio.microsoft.com/downloads/
2. 下载 "Visual Studio 2022 Community"
3. 运行安装程序
4. 选择工作负载: "使用C++的桌面开发" (Desktop development with C++)
5. 在右侧单个组件中确保勾选:
   ☑ MSVC v143 - VS 2022 C++ x64/x86 生成工具
   ☑ Windows 11 SDK (10.0.22621.0) 或更新版本
   ☑ C++ CMake工具
6. 点击安装（需要约5-10GB磁盘空间）
7. 安装完成后重启电脑
```

**选项B: 轻量级 (Build Tools only)**
```
1. 访问 https://visualstudio.microsoft.com/visual-cpp-build-tools/
2. 下载 "Build Tools for Visual Studio 2022"
3. 运行安装程序
4. 勾选 "C++ 生成工具"
5. 确保包含:
   ☑ MSVC v143 编译器
   ☑ Windows 11 SDK
6. 安装完成
```

**验证安装:**
```powershell
# 打开 PowerShell 或 CMD
cl  # 应该显示版本信息
# 如果提示找不到命令，重启后重试
```

---

### 第2步：安装 Git 和 CMake

#### Git:
```powershell
# 方法A: 使用 winget (Windows 10/11自带)
winget install Git.Git

# 方法B: 手动下载
# https://git-scm.com/download/win
# 运行安装程序，一路默认即可

# 验证
git --version
# 输出: git version 2.x.x
```

#### CMake:
```powershell
# 方法A: winget
winget install Kitware.Cmake

# 方法B: 手动下载
# https://cmake.org/download/
# 下载 cmake-x.x.x-windows-x86_64.msi 并安装

# 验证
cmake --version
# 输出: cmake version 3.x.x
```

---

### 第3步：配置 vcpkg (包管理器)

```bash
# 1. 克隆 vcpkg 到固定位置（建议放在C盘根目录）
cd C:\
git clone https://github.com/microsoft/vcpkg.git

# 2. 初始化 vcpkg (需要几分钟)
cd vcpkg
bootstrap-vcpkg.bat

# 3. 设置环境变量 (永久生效)
# 右键 "此电脑" → 属性 → 高级系统设置 → 环境变量 → 系统变量 → 新建:
#   变量名: VCPKG_ROOT
#   变量值: C:\vcpkg

# 4. 安装Aviateur所需的所有依赖 (耗时10-20分钟)
vcpkg install libusb:x64-windows ffmpeg:x64-windows opencv4:x64-windows libsodium:x64-windows sdl2:x64-windows nlohmann-json:x64-windows
```

**常见问题：**
- 如果报错 "找不到Python"，安装Python 3.8+并添加到PATH
- 如果网络慢，可以设置代理: `set HTTP_PROXY=http://your-proxy:port`
- 如果中断了，重新执行同一命令会继续（vcpkg支持断点续传）

---

### 第4步：获取 Aviateur 源码

```bash
# 创建工作目录
mkdir C:\dev\aviateur_build
cd C:\dev\aviateur_build

# 克隆Aviateur源码 (指定v0.1.2版本)
git clone -b 0.1.2 --recursive https://github.com/OpenIPC/aviateur.git

# 进入源码目录
cd aviateur

# 更新子模块 (确保第三方库完整)
git submodule update --init --recursive
```

**目录结构应该是：**
```
C:\dev\aviateur_build\aviateur\
├── src/
│   ├── feature/
│   │   └── (现有模块)
│   ├── gui/
│   ├── player/
│   └── wifi/
├── 3rd/              # 第三方库 (子模块)
│   ├── json/         # nlohmann/json
│   ├── devourer/
│   ├── revector/
│   └── sdl/
├── assets/
├── CMakeLists.txt    # 主构建文件 ← 需要修改
└── README.md
```

---

### 第5步：集成 Car Control 模块

#### 5.1 复制代码文件

```bash
# 创建car_control目录
mkdir src\feature\car_control

# 从你的Pycharm项目复制文件
copy "c:\Users\admin\PycharmProjects\pythonProject\aviateur_car_control\car_control_module.h" src\feature\car_control\
copy "c:\Users\admin\PycharmProjects\pythonProject\aviateur_car_control\car_control_module.cpp" src\feature\car_control\
```

#### 5.2 创建模块CMakeLists.txt

在 `src\feature\car_control\` 目录下创建文件 `CMakeLists.txt`:

```cmake
# Car Control Module - CMake Configuration
add_library(car_control STATIC
    car_control_module.cpp
)

target_include_directories(car_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd/json/single_include/nlohmann
)

target_link_libraries(car_control PRIVATE
    ws2_32
)

message(STATUS "Car Control Module: ENABLED")
```

#### 5.3 修改主CMakeLists.txt

打开 `C:\dev\aviateur_build\aviateur\CMakeLists.txt`，找到类似这样的位置（通常在文件末尾附近）：

```cmake
# 找到这一行附近:
target_link_libraries(aviateur PRIVATE
    # ... 其他库 ...
)

# 在它后面添加:
# ====== Car Control Module ======
option(ENABLE_CAR_CONTROL "Enable undercarriage vehicle control" ON)
if(ENABLE_CAR_CONTROL)
    add_subdirectory(src/feature/car_control)
    target_link_libraries(aviateur PRIVATE car_control)
    if(WIN32)
        target_link_libraries(car_control PRIVATE ws2_32)
    endif()
endif()
```

**或者如果你不想手动编辑，可以使用我提供的PowerShell脚本自动修改。**

---

### 第6步：编译项目

```bash
# 进入build目录
mkdir build
cd build

# 配置项目 (指定vcpkg工具链和VS2022)
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DENABLE_CAR_CONTROL=ON

# 开始编译 (Release模式，多线程加速)
cmake --build . --config Release --parallel
```

**编译时间：**
- 首次完整编译: 5-15分钟
- 后续增量编译: 1-3分钟

**输出位置：**
```
C:\dev\aviateur_build\aviateur\build\bin\Release\aviateur.exe
```

---

### 第7步：测试运行

```bash
# 直接双击exe运行
start "" "C:\dev\aviateur_build\aviateur\build\bin\Release\aviateur.exe"

# 或者从CMD/Powershell运行
.\bin\Release\aviateur.exe
```

**预期界面：**
```
┌─────────────────────────────────────────────────────┐
│  Aviateur v0.1.2                                  │
├────────────────────────────┬────────────────────────┤
│                            │                        │
│     📹 视频显示区域        │  🆕 Car Control       │
│     (原有功能)             │                        │
│                            │  [IP输入框] [Connect]  │
│                            │  Mode: STANDBY        │
│                            │  Distance: 0.0m       │
│                            │                        │
│                            │     ○ (虚拟摇杆)      │
│                            │                        │
│                            │  [⛑STOP] [MODE] [LED] │
├────────────────────────────┴────────────────────────┤
│  [Adapter▼] [Channel] [Key] │ [Record] [Settings⚙]   │
└─────────────────────────────────────────────────────┘
```

看到右侧有 **Car Control** 面板就说明集成成功了！🎉

---

## 🐛 常见编译错误及解决方案

### 错误1: "找不到 Windows SDK"

**现象：**
```
CMake Error: Could not find any instance of Visual Studio
```

**解决：**
```
1. 打开 Visual Studio Installer
2. 点击 "修改"
3. 切换到 "单个组件" 标签
4. 勾选 "Windows 11 SDK" (最新版本)
5. 点击修改/安装
6. 重启电脑后重新运行cmake
```

### 错误2: "找不到 vcpkg.cmake"

**现象：**
```
CMake Error: Could not find CMAKE_TOOLCHAIN_FILE
```

**解决：**
```bash
# 确认vcpkg路径正确
dir C:\vcpkg\scripts\buildsystems\vcpkg.cmake

# 如果不存在，重新初始化vcpkg
cd C:\vcpkg
bootstrap-vcpkg.bat
```

### 错误3: "无法解析的外部符号 _imp_*"

**现象：**
```
LNK2019: unresolved external symbol __imp_*
```

**原因：** 缺少某个依赖库

**解决：**
```bash
# 确认所有依赖都已安装
vcpkg list | findstr "libusb ffmpeg opencv libsodium sdl2"

# 如果缺少某个库，重新安装
vcpkg install <缺失库名>:x64-windows
```

### 错误4: "nlohmann/json not found"

**现象：**
```
fatal error: nlohmann/json.hpp: No such file or directory
```

**解决：**
```bash
# 方案A: 通过vcpkg安装 (推荐)
vcpkg install nlohmann-json:x64-windows

# 方案B: 手动下载头文件
# 从 https://github.com/nlohmann/json/releases 下载
# 解压到 aviateur/3rd/json/single_include/nlohmann/
```

### 错误5: "ws2_32 未定义"

**现象：**
```
LNK2019: unresolved external symbol __imp_closesocket
```

**解决：**
确认 `target_link_libraries` 中包含 `ws2_32` (Winsock库)。检查 car_control 的 CMakeLists.txt 是否正确。

---

## 📊 编译优化建议

### 加速编译 (多核CPU)

```bash
# 使用所有CPU核心并行编译
cmake --build . --config Release --parallel $(nproc)  # Linux
cmake --build . --config Release --parallel          # Windows (自动检测)
```

### 减少编译输出大小

```bash
# Release模式已经优化过，但还可以进一步 Strip 符号表
# 在CMakeLists.txt中添加:
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-")
```

### 只编译CarControl相关部分

```bash
# 只重新编译修改过的目标
cmake --build . --config Release --target car_control aviateur
```

---

## 🔄 后续开发工作流

修改代码后的标准流程：

```bash
# 1. 编辑源码 (用VS Code或其他编辑器)
code C:\dev\aviateur_build\aviateur\src\feature\car_control\car_control_module.cpp

# 2. 重新编译 (只编译变更的部分)
cd C:\dev\aviateur_build\aviateur\build
cmake --build . --config Release

# 3. 测试运行
.\bin\Release\aviateur.exe

# 4. 如需调试，可以在VS中打开项目
#    File → Open → CMake → 选择 CMakeLists.txt
#    设置断点，F5启动调试
```

---

## 📁 最终文件清单

编译完成后，你的工作目录应该如下：

```
C:\dev\aviateur_build\
├── aviateur\                    # Aviateur源码
│   ├── build\                   # 编译输出
│   │   └── bin\Release\
│   │       └── aviateur.exe  ← ★ 这就是最终可执行文件!
│   ├── src\
│   │   └── feature\car_control\  ← ★ 你添加的小车控制模块
│   │       ├── car_control_module.h
│   │       ├── car_control_module.cpp
│   │       └── CMakeLists.txt
│   └── CMakeLists.txt           # 已修改，启用Car Control
│
├── vcpkg\                      # 包管理器 (可复用)
│   └── installed\x64-windows\   # 已安装的依赖库
│
└── setup_and_build.bat         # 一键搭建脚本 (可选保留)
```

---

## ✅ 验证清单

编译完成后，请逐项检查：

- [ ] `aviateur.exe` 文件存在且能正常运行
- [ ] 启动后能看到右侧新增的 "Car Control" 面板
- [ ] 面板包含: IP输入框、Connect按钮、状态显示区、虚拟摇杆、急停按钮
- [ ] 输入小车IP点击Connect后，状态灯变绿
- [ ] 拖动摇杆时，小车端终端显示收到 cmd_vel 消息
- [ ] 按 WASD 键也能控制小车移动
- [ ] 点击急停按钮，小车立即停止

全部通过？🎉 **恭喜你，已经拥有了一个带完整控制功能的图传软件！**

---

**文档版本**: v1.0  
**适用系统**: Windows 10/11 (x64)  
**最后更新**: 2026-05-06

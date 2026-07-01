@echo off
REM ==================================================================================
REM  Aviateur + Car Control - Windows 一键环境搭建与编译脚本
REM ==================================================================================
REM  
REM  功能：
REM    1. 检测并安装必要工具 (Git, CMake, Visual Studio)
REM    2. 安装 vcpkg 包管理器并编译依赖库
REM    3. 克隆 Aviateur 源码及子模块
REM    4. 复制小车控制模块到正确位置
REM    5. 编译生成带控制功能的 aviateur.exe
REM  
REM  使用方法：
REM    右键 → 以管理员身份运行此脚本
REM    或在管理员PowerShell中执行: .\setup_and_build.bat
REM  
REM  预计时间：首次运行约30-60分钟（取决于网速）
REM         后续编译约5-10分钟
REM
REM  Author: AI Assistant
REM  Date: 2026-05-06
REM ==================================================================================

echo.
echo ================================================================================
echo   Aviateur + Car Control - 自动化构建系统
echo ================================================================================
echo.

REM ====== 检查管理员权限 ======
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] 请以管理员身份运行此脚本！
    echo        右键点击此文件 → 以管理员身份运行
    pause
    exit /b 1
)
echo [OK] 管理员权限确认

REM ====== 设置工作目录 ======
set "WORK_DIR=C:\dev\aviateur_build"
set "VCPKG_DIR=C:\dev\vcpkg"
set "AVIATEUR_DIR=%WORK_DIR%\aviateur"
set "CAR_CONTROL_SRC=%USERPROFILE%\PycharmProjects\pythonProject\aviateur_car_control"

echo [INFO] 工作目录: %WORK_DIR%
echo.

REM ====== Step 1: 检查/安装 Git ======
echo [Step 1/7] 检查 Git...
where git >nul 2>&1
if %errorLevel% neq 0 (
    echo [INSTALL] 正在安装 Git...
    winget install Git.Git --accept-package-agreements --accept-source-agreements
    if %errorLevel% neq 0 (
        echo [ERROR] Git安装失败，请手动安装: https://git-scm.com/download/win
        pause
        exit /b 1
    )
    set "PATH=%PATH%;C:\Program Files\Git\cmd"
) else (
    git --version
)
echo [OK] Git 已就绪

REM ====== Step 2: 检查/安装 CMake ======
echo.
echo [Step 2/7] 检查 CMake...
where cmake >nul 2>&1
if %errorLevel% neq 0 (
    echo [INSTALL] 正在安装 CMake...
    winget install Kitware.CMake --accept-package-agreements --accept-source-agreements
    if %errorLevel% neq 0 (
        echo [ERROR] CMake安装失败，请手动安装: https://cmake.org/download/
        pause
        exit /b 1
    )
    set "PATH=%PATH%;C:\Program Files\CMake\bin"
) else (
    cmake --version | findstr /C:"cmake version"
)
echo [OK] CMake 已就绪

REM ====== Step 3: 检查 Visual Studio Build Tools ======
echo.
echo [Step 3/7] 检查 Visual Studio Build Tools...
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC" (
    echo [OK] Visual Studio 2022 已安装
    set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
    goto :vs_found
)

if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC" (
    echo [OK] VS2022 Build Tools 已安装
    set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\BuildTools"
    goto :vs_found
)

echo [WARN] 未检测到Visual Studio，正在尝试安装Build Tools...
echo        这将下载约1.5GB的组件，需要几分钟...

winget install Microsoft.VisualStudio.2022.BuildTools --override "--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --quiet --wait"
if %errorLevel% equ 0 (
    echo [OK] Visual Studio Build Tools 安装成功
    set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\BuildTools"
) else (
    echo [ERROR] Visual Studio安装失败！
    echo.
    echo 请手动执行以下操作之一:
    echo   方式A: 下载并安装 Visual Studio 2022 Community (免费)
    echo          https://visualstudio.microsoft.com/downloads/
    echo          安装时勾选 "使用C++的桌面开发"
    echo.
    echo   方式B: 下载 Build Tools for Visual Studio 2022 (较小)
    echo          https://visualstudio.microsoft.com/visual-cpp-build-tools/
    echo          勾选 "MSVC v143编译器" 和 "Windows 11 SDK"
    echo.
    pause
    exit /b 1
)

:vs_found
echo [OK] 编译器已就绪: %VS_PATH%

REM ====== Step 4: 安装配置 vcpkg ======
echo.
echo [Step 4/7] 配置 vcpkg 包管理器...

if not exist "%VCPKG_DIR%" (
    echo [CLONE] 克隆 vcpkg...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_DIR%"
    if %errorLevel% neq 0 (
        echo [ERROR] vcpkg克隆失败，检查网络连接
        pause
        exit /b 1
    )
    
    echo [BOOTSTRAP] 初始化 vcpkg...
    call "%VCPKG_DIR%\bootstrap-vcpkg.bat"
    if %errorLevel% neq 0 (
        echo [ERROR] vcpkg初始化失败
        pause
        exit /b 1
    )
) else (
    echo [OK] vcpkg 已存在: %VCPKG_DIR%
)

REM ====== Step 5: 安装依赖库 ======
echo.
echo [Step 5/7] 安装依赖库 (libusb, ffmpeg, opencv, libsodium, sdl2)...
echo        这可能需要10-20分钟，请耐心等待...

call "%VCPKG_DIR%\vcpkg.exe" install libusb:x64-windows ffmpeg:x64-windows opencv4:x64-windows libsodium:x64-windows sdl2:x64-windows nlohmann-json:x64-windows
if %errorLevel% neq 0 (
    echo [ERROR] 依赖库安装失败！
    echo        可能原因: 网络问题或缺少Visual Studio组件
    pause
    exit /b 1
)
echo [OK] 所有依赖库安装完成

REM ====== Step 6: 克隆 Aviateur 源码 ======
echo.
echo [Step 6/7] 获取 Aviateur 源码...

if not exist "%AVIATEUR_DIR%" (
    mkdir "%WORK_DIR%" 2>nul
    
    echo [CLONE] 克隆 Aviateur v0.1.2...
    git clone -b 0.1.2 --recursive https://github.com/OpenIPC/aviateur.git "%AVIATEUR_DIR%"
    if %errorLevel% neq 0 (
        echo [ERROR] Aviateur克隆失败
        pause
        exit /b 1
    )
    
    REM 更新子模块
    cd /d "%AVIATEUR_DIR%"
    git submodule update --init --recursive
    echo [OK] Aviateur源码获取完成
) else (
    echo [OK] Aviateur源码已存在: %AVIATEUR_DIR%
)

REM ====== Step 7: 集成小车控制模块 ======
echo.
echo [Step 7/7] 集成 Car Control 模块...

REM 创建目标目录
if not exist "%AVIATEUR_DIR%\src\feature\car_control" (
    mkdir "%AVIATEUR_DIR%\src\feature\car_control"
)

REM 复制文件
echo [COPY] 复制 car_control_module.h ...
copy /Y "%CAR_CONTROL_SRC%\car_control_module.h" "%AVIATEUR_DIR%\src\feature\car_control\" >nul

echo [COPY] 复制 car_control_module.cpp ...
copy /Y "%CAR_CONTROL_SRC%\car_control_module.cpp" "%AVIATEUR_DIR%\src\feature\car_control\" >nul

REM 创建模块的CMakeLists.txt
echo [GEN] 生成 car_control\CMakeLists.txt ...
(
echo # Car Control Module - CMake Configuration
echo add_library^(car_control STATIC
echo     car_control_module.cpp
echo ^)
echo.
echo target_include_directories^(car_control PUBLIC
echo     ^${CMAKE_CURRENT_SOURCE_DIR}
echo     ^${CMAKE_CURRENT_SOURCE_DIR}/../../3rd/json/single_include/nlohmann
echo ^)
echo.
echo target_link_libraries^(car_control PRIVATE
echo     ws2_32
echo ^)
echo.
echo message^(STATUS "Car Control Module: ENABLED"^)
) > "%AVIATEUR_DIR%\src\feature\car_control\CMakeLists.txt"

REM 修改主CMakeLists.txt添加模块
echo [PATCH] 修改主CMakeLists.txt启用Car Control...

cd /d "%AVIATEUR_DIR%"

REM 创建备份
copy /Y CMakeLists.txt CMakeLists.txt.backup >nul 2>&1

REM 使用PowerShell修改CMakeLists.txt（在合适位置插入）
powershell -Command ^
    "$content = Get-Content 'CMakeLists.txt' -Raw; " ^
    "$insert = @'
echo.
echo # ====== Car Control Module ======
echo option(ENABLE_CAR_CONTROL \"Enable undercarriage vehicle control\" ON^)
echo if^(ENABLE_CAR_CONTROL^)
echo     add_subdirectory^(src/feature/car_control^)
echo     target_link_libraries^(aviateur PRIVATE car_control^)
echo     if^(WIN32^)
echo         target_link_libraries^(car_control PRIVATE ws2_32^)
     endif(^)
echo endif(^)
echo '@; " ^
    "if ($content -match 'target_link_libraries.*aviateur') { " ^
    "    $content = $content -replace '(target_link_libraries\(aviateur[^)]*\))', "`$1`n$insert"; " ^
    "} elseif ($content -match '# Testing') { " ^
    "    $content = $content -replace '# Testing', \"$insert`n# Testing\"; " ^
    "} else { " ^
    "    $content += \"`n$insert\"; " ^
    "} " ^
    "Set-Content -Path 'CMakeLists.txt' -Value $content -NoNewline"

echo [OK] Car Control 模块集成完成！

REM ====== 开始编译 ======
echo.
echo ================================================================================
echo   开始编译 Aviateur + Car Control
echo ================================================================================
echo.

if not exist "%AVIATEUR_DIR%\build" (
    mkdir build
)

cd /d "%AVIATEUR_DIR%\build"

echo [CONFIG] 配置CMake...
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="%VCPKG_DIR%\scripts\buildsystems\vcpkg.cmake" -DENABLE_CAR_CONTROL=ON
if %errorLevel% neq 0 (
    echo [ERROR] CMake配置失败！检查上方错误信息
    pause
    exit /b 1
)

echo.
echo [BUILD] 编译中... (这可能需要5-15分钟)
cmake --build . --config Release --parallel
if %errorLevel% neq 0 (
    echo.
    echo [ERROR] 编译失败！常见原因:
    echo   1. 缺少Windows SDK - 在VS Installer中安装 "Windows 11 SDK"
    echo   2. 缺少C++工具集 - 确保安装了 "MSVC v143 编译器"
    echo   3. 路径问题 - 确保路径不含中文或空格
    echo.
    pause
    exit /b 1
)

echo.
echo ================================================================================
echo   ✅ 编译成功！
echo ================================================================================
echo.
echo   输出文件: %AVIATEUR_DIR%\build\bin\Release\aviateur.exe
echo.
echo   下一步:
echo   1. 双击运行 aviateur.exe
echo   2. 在右侧应该能看到 "Car Control" 控制面板
echo   3. 输入小车IP地址 (如 192.168.1.100) 并点击 Connect
echo   4. 使用虚拟摇杆或键盘 WASD 控制小车
echo.
echo   如需重新编译，只需运行:
echo      cd %AVIATEUR_DIR%\build
echo      cmake --build . --config Release
echo.

REM 可选：复制到桌面快捷方式
set "EXE_PATH=%AVIATEUR_DIR%\build\bin\Release\aviateur.exe"
if exist "%EXE_PATH%" (
    copy /Y "%EXE_PATH%" "%USERPROFILE%\Desktop\Aviateur_CarControl.exe" >nul 2>&1
    echo [INFO] 已创建桌面快捷方式: Aviateer_CarControl.exe
)

pause

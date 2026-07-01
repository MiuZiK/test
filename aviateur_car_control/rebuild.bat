@echo off
REM ==================================================================================
REM  Aviateur 快速重编译脚本 (环境已就绪时使用)
REM ==================================================================================
REM  
REM  前提: 已完成 setup_and_build.bat 的首次运行，或手动配置好所有依赖
REM  
REM  用途:
REM    修改源码后快速重新编译 (1-3分钟)
REM    不需要重新安装任何东西
REM
REM ==================================================================================

echo.
echo ================================================================================
echo   Aviateur - Quick Rebuild
echo ================================================================================
echo.

set "AVIATEUR_DIR=C:\dev\aviateur_build\aviateur"
set "VCPKG_DIR=C:\vcpkg"

REM 检查目录是否存在
if not exist "%AVIATEUR_DIR%" (
    echo [ERROR] Aviateur源码目录不存在!
    echo        请先运行 setup_and_build.bat 完成初始搭建
    pause
    exit /b 1
)

if not exist "%AVIATEUR_DIR%\build" (
    mkdir "%AVIATEUR_DIR%\build"
)

cd /d "%AVIATEUR_DIR%\build"

echo [CONFIG] CMake configure...
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="%VCPKG_DIR%\scripts\buildsystems\vcpkg.cmake" -DENABLE_CAR_CONTROL=ON
if %errorLevel% neq 0 (
    echo [ERROR] CMake failed!
    pause
    exit /b 1
)

echo.
echo [BUILD] Compiling...
cmake --build . --config Release --parallel

if %errorLevel% equ 0 (
    echo.
    echo ================================================================================
    echo   ✅ Build Successful!
    echo ================================================================================
    echo.
    echo   Output: %AVIATEUR_DIR%\build\bin\Release\aviateur.exe
    
    REM 直接运行
    echo.
    echo [LAUNCH] Starting aviateur...
    start "" "%AVIATEUR_DIR%\build\bin\Release\aviateur.exe"
) else (
    echo.
    echo [ERROR] Build failed! Check errors above.
)

pause

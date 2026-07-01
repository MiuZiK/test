@echo off
chcp 65001 >nul 2>&1
REM ==================================================================================
REM  Aviateur + Car Control - Automated Build Script (English Version)
REM ==================================================================================
REM.
echo ================================================================================
echo   Aviateur + Car Control - Build System
echo ================================================================================
echo.

REM ====== Check Admin ======
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] Please run as Administrator!
    echo        Right-click this file -^> Run as administrator
    pause
    exit /b 1
)
echo [OK] Admin rights confirmed

REM ====== Setup Paths ======
set "WORK_DIR=C:\dev\aviateur_build"
set "VCPKG_DIR=C:\vcpkg"
set "AVIATEUR_DIR=%WORK_DIR%\aviateur"
set "CAR_CONTROL_SRC=%USERPROFILE%\PycharmProjects\pythonProject\aviateur_car_control"

echo [INFO] Work directory: %WORK_DIR%
echo.

REM ====== Step 1: Check Git ======
echo [Step 1/7] Checking Git...
where git >nul 2>&1
if %errorLevel% neq 0 (
    echo [INSTALLING] Git...
    winget install Git.Git --accept-package-agreements --accept-source-agreements --silent
    set "PATH=%PATH%;C:\Program Files\Git\cmd"
) else (
    git --version
)
echo [OK] Git ready

REM ====== Step 2: Check CMake ======
echo.
echo [Step 2/7] Checking CMake...
where cmake >nul 2>&1
if %errorLevel% neq 0 (
    echo [INSTALLING] CMake...
    winget install Kitware.Cmake --accept-package-agreements --accept-source-agreements --silent
    set "PATH=%PATH%;C:\Program Files\CMake\bin"
) else (
    cmake --version | findstr /C:"cmake version"
)
echo [OK] CMake ready

REM ====== Step 3: Check Visual Studio ======
echo.
echo [Step 3/7] Checking Visual Studio...

set "VS_FOUND=0"

if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC" (
    echo [OK] Visual Studio 2022 Community found
    set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
    set "VS_FOUND=1"
    goto :vs_ok
)

if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC" (
    echo [OK] VS2022 Build Tools found
    set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\BuildTools"
    set "VS_FOUND=1"
    goto :vs_ok
)

if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC" (
    echo [OK] VS2022 Build Tools found (x86)
    set "VS_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
    set "VS_FOUND=1"
    goto :vs_ok
)

if %VS_FOUND%==0 (
    echo [WARN] Visual Studio not detected
    echo        Since you mentioned VS is installed, let me search more...
    
    dir /s /b "C:\Program Files*\Microsoft Visual Studio\2022\*\VC\Tools\MSVC" >nul 2>&1
    if %errorLevel% equ 0 (
        for /f "delims=" %%i in ('dir /s /b "C:\Program Files*\Microsoft Visual Studio\2022\*\VC\Tools\MSVC" 2^>nul') do (
            set "VS_PATH=%%~dpi.."
            echo [OK] Found VS at: !VS_PATH!"
            set "VS_FOUND=1"
            goto :vs_ok
        )
    )
)

:vs_ok

if %VS_FOUND%==0 (
    echo.
    echo [ERROR] Cannot find Visual Studio 2022!
    echo.
    echo Please verify:
    echo   1. Open Visual Studio Installer
    echo   2. Make sure "Desktop development with C++" workload is installed
    echo   3. Or install Build Tools from:
    echo      https://visualstudio.microsoft.com/visual-cpp-build-tools/
    echo.
    pause
    exit /b 1
)
echo [OK] Compiler ready

REM ====== Step 4: Setup vcpkg ======
echo.
echo [Step 4/7] Setting up vcpkg package manager...

if not exist "%VCPKG_DIR%" (
    echo [CLONING] vcpkg from GitHub...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_DIR%"
    if %errorLevel% neq 0 (
        echo [ERROR] Failed to clone vcpkg. Check network connection.
        pause
        exit /b 1
    )
    
    echo [BOOTSTRAP] Initializing vcpkg...
    call "%VCPKG_DIR%\bootstrap-vcpkg.bat"
    if %errorLevel% neq 0 (
        echo [ERROR] vcpkg bootstrap failed
        pause
        exit /b 1
    )
) else (
    echo [OK] vcpkg already exists: %VCPKG_DIR%
)

REM ====== Step 5: Install Dependencies ======
echo.
echo [Step 5/7] Installing dependencies via vcpkg...
echo        This may take 10-20 minutes, please wait...

call "%VCPKG_DIR%\vcpkg.exe" install libusb:x64-windows ffmpeg:x64-windows opencv4:x64-windows libsodium:x64-windows sdl2:x64-windows nlohmann-json:x64-windows
if %errorLevel% neq 0 (
    echo [ERROR] Dependency installation failed!
    echo        Possible causes: network issue or missing VS components
    pause
    exit /b 1
)
echo [OK] All dependencies installed

REM ====== Step 6: Clone Aviateur Source ======
echo.
echo [Step 6/7] Getting Aviateur source code...

if not exist "%AVIATEUR_DIR%" (
    mkdir "%WORK_DIR%" 2>nul
    
    echo [CLONING] Aviateur v0.1.2...
    git clone -b 0.1.2 --recursive https://github.com/OpenIPC/aviateur.git "%AVIATEUR_DIR%"
    if %errorLevel% neq 0 (
        echo [ERROR] Failed to clone Aviateur
        pause
        exit /b 1
    )
    
    cd /d "%AVIATEUR_DIR%"
    git submodule update --init --recursive
    echo [OK] Source code downloaded
) else (
    echo [OK] Source already exists: %AVIATEUR_DIR%
)

REM ====== Step 7: Integrate Car Control Module ======
echo.
echo [Step 7/7] Integrating Car Control module...

if not exist "%AVIATEUR_DIR%\src\feature\car_control" (
    mkdir "%AVIATEUR_DIR%\src\feature\car_control"
)

echo [COPY] car_control_module.h ...
copy /Y "%CAR_CONTROL_SRC%\car_control_module.h" "%AVIATEUR_DIR%\src\feature\car_control\" >nul

echo [COPY] car_control_module.cpp ...
copy /Y "%CAR_CONTROL_SRC%\car_control_module.cpp" "%AVIATEUR_DIR%\src\feature\car_control\" >nul

echo [GEN] car_control\CMakeLists.txt ...
(
echo # Car Control Module Configuration
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

echo [PATCH] Modifying main CMakeLists.txt...

cd /d "%AVIATEUR_DIR%"

copy /Y CMakeLists.txt CMakeLists.txt.backup >nul 2>&1

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "$content = Get-Content 'CMakeLists.txt' -Raw; " ^
    "$insert = @'
echo.
echo # Car Control Module
echo option(ENABLE_CAR_CONTROL \"Enable vehicle control\" ON^)
echo if^(ENABLE_CAR_CONTROL^)
echo     add_subdirectory^(src/feature/car_control^)
echo     target_link_libraries^(aviateur PRIVATE car_control^)
echo     if^(WIN32^)
echo         target_link_libraries^(car_control PRIVATE ws2_32^)
     endif(^)
echo endif(^)
echo '@; " ^
    "if ($content -match 'target_link_libraries.*aviateur.*PRIVATE') { " ^
    "    $content = $content -replace '(target_link_libraries\(aviateur PRIVATE[^)]*\))', \"`$1`n$insert\"; " ^
    "} else { " ^
    "    $content += \"`n$insert\"; " ^
    "} " ^
    "Set-Content -Path 'CMakeLists.txt' -Value $content -NoNewline"

echo [OK] Car Control module integrated!

REM ====== Build ======
echo.
echo ================================================================================
echo   Building Aviateur + Car Control
echo ================================================================================
echo.

if not exist "%AVIATEUR_DIR%\build" (
    mkdir build
)

cd /d "%AVIATEUR_DIR%\build"

echo [CONFIG] Running CMake...
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="%VCPKG_DIR%\scripts\buildsystems\vcpkg.cmake" -DENABLE_CAR_CONTROL=ON
if %errorLevel% neq 0 (
    echo [ERROR] CMake configuration failed!
    echo.
    echo Common fixes:
    echo   1. Install Windows SDK from VS Installer
    echo   2. Ensure MSVC v143 compiler is installed
    echo   3. Avoid spaces in file paths
    echo.
    pause
    exit /b 1
)

echo.
echo [BUILD] Compiling... ^(~5-15 minutes^)
cmake --build . --config Release --parallel
if %errorLevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    echo.
    pause
    exit /b 1
)

echo.
echo ================================================================================
echo   SUCCESS! Build completed!
echo ================================================================================
echo.
echo   Output: %AVIATEUR_DIR%\build\bin\Release\aviateur.exe
echo.
echo   Next steps:
echo   1. Double-click aviateur.exe to launch
echo   2. Look for "Car Control" panel on the right side
echo   3. Enter your robot IP ^(e.g., 192.168.1.100^) and click Connect
echo   4. Use virtual joystick or WASD keys to control
echo.

set "EXE_PATH=%AVIATEUR_DIR%\build\bin\Release\aviateur.exe"
if exist "%EXE_PATH%" (
    copy /Y "%EXE_PATH%" "%USERPROFILE%\Desktop\Aviateur_CarControl.exe" >nul 2>&1
    echo [INFO] Desktop shortcut created: Aviateer_CarControl.exe
)

pause

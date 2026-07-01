/**
 * ==================================================================================
 * Aviateur 小车控制模块 - 核心实现
 * ==================================================================================
 * 
 * 本文件包含集成到Aviateur所需的所有核心组件：
 *  1. ROSBridge - TCP通信桥接（连接小车ROS系统）
 *  2. VirtualJoystick - 虚拟摇杆UI控件
 *  3. CarControlPanel - 控制面板主界面
 *  4. StatusDisplay - 状态显示组件
 * 
 * 集成方式：
 *  - 将此目录复制到 aviateur/src/feature/car_control/
 *  - 在 CMakeLists.txt 中添加: add_subdirectory(src/feature/car_control)
 *  - 在 gui_interface.h 中包含头文件并初始化
 * 
 * Author: AI Assistant  
 * Date: 2026-05-06
 * Based on: OpenIPC/aviateur v0.1.2
 */

#ifndef CAR_CONTROL_MODULE_H
#define CAR_CONTROL_MODULE_H

// ==================== 平台检测 ====================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef SOCKET socket_t;
    #define INVALID_SOCKET_VAL INVALID_SOCKET
    #define CLOSE_SOCKET closesocket
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int socket_t;
    #define INVALID_SOCKET_VAL -1
    #define CLOSE_SOCKET close
#endif

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <queue>
#include <chrono>
#include <vector>
#include <cmath>

// JSON库 (nlohmann/json 已在 Aviateur 的 3rdparty 中)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Revector UI框架头文件 (Aviateur自带)
// 注意：实际编译时路径可能需要调整
#ifdef AVIATEUR_INTEGRATION
    #include "revector/widget.h"
    #include "revector/panel.h"
    #include "revector/button.h"
    #include "revector/text_input.h"
    #include "revector/label.h"
#endif


// ==================== 消息类型定义 ====================

namespace car_control {

/**
 * 系统状态数据结构
 */
struct SystemStatus {
    std::string mode = "STANDBY";       // 当前模式
    double battery_voltage = 0.0;        // 电池电压(V)
    double uptime = 0.0;                // 运行时间(秒)
    bool connected = false;             // 是否已连接
    int cpu_load = 0;                   // CPU负载(%)
    double temperature = 0.0;           // 温度(°C)
    
    json to_json() const {
        return {
            {"mode", mode},
            {"battery_voltage", battery_voltage},
            {"uptime", uptime},
            {"connected", connected},
            {"cpu_load", cpu_load},
            {"temperature", temperature}
        };
    }
    
    static SystemStatus from_json(const json& j) {
        SystemStatus s;
        if (j.contains("mode")) s.mode = j["mode"];
        if (j.contains("battery_voltage")) s.battery_voltage = j["battery_voltage"];
        if (j.contains("uptime")) s.uptime = j["uptime"];
        if (j.contains("connected")) s.connected = j["connected"];
        if (j.contains("cpu_load")) s.cpu_load = j["cpu_load"];
        if (j.contains("temperature")) s.temperature = j["temperature"];
        return s;
    }
};

/**
 * 里程计数据
 */
struct OdometryData {
    double x = 0, y = 0;               // 位置(m)
    double yaw = 0.0;                  // 航向角(rad)
    double linear_velocity = 0.0;      // 线速度(m/s)
    double angular_velocity = 0.0;     // 角速度(rad/s)
    double traveled_distance = 0.0;   // 累计行驶距离(m)
    
    json to_json() const {
        return {
            {"position", {{"x", x}, {"y", y}}},
            {"orientation", {{"yaw", yaw}}},
            {"linear_velocity", linear_velocity},
            {"angular_velocity", angular_velocity},
            {"traveled_distance", traveled_distance}
        };
    }
    
    static OdometryData from_json(const json& j) {
        OdometryData o;
        if (j.contains("position")) {
            o.x = j["position"].value("x", 0.0);
            o.y = j["position"].value("y", 0.0);
        }
        if (j.contains("orientation")) {
            o.yaw = j["orientation"].value("yaw", 0.0);
        }
        if (j.contains("linear_velocity")) o.linear_velocity = j["linear_velocity"];
        if (j.contains("angular_velocity")) o.angular_velocity = j["angular_velocity"];
        if (j.contains("traveled_distance")) o.traveled_distance = j["traveled_distance"];
        return o;
    }
};

/**
 * 轨道检测状态
 */
struct RailStatus {
    bool detected = false;              // 是否检测到轨道
    double confidence = 0.0;            // 置信度[0,1]
    double rail_width = 0.0;           // 轨距(m)
    double cross_track_error = 0.0;    // 横向误差(m)
    int inlier_count_left = 0;          // 左轨内点数
    int inlier_count_right = 0;         // 右轨内点数
    
    json to_json() const {
        return {
            {"detected", detected},
            {"confidence", confidence},
            {"rail_width", rail_width},
            {"cross_track_error", cross_track_error},
            {"inlier_count_left", inlier_count_left},
            {"inlier_count_right", inlier_count_right}
        };
    }
    
    static RailStatus from_json(const json& j) {
        RailStatus r;
        if (j.contains("detected")) r.detected = j["detected"];
        if (j.contains("confidence")) r.confidence = j["confidence"];
        if (j.contains("rail_width")) r.rail_width = j["rail_width"];
        if (j.contains("cross_track_error")) r.cross_track_error = j["cross_track_error"];
        if (j.contains("inlier_count_left")) r.inlier_count_left = j["inlier_count_left"];
        if (j.contains("inlier_count_right")) r.inlier_count_right = j["inlier_count_right"];
        return r;
    }
};

/**
 * 控制命令类型枚举
 */
enum class CommandType {
    CMD_VEL,           // 速度控制
    MODE_CHANGE,       // 模式切换
    EMERGENCY_STOP,    // 急停
    GIMBAL_CMD,        // 云台控制
    LED_TOGGLE,        // LED开关
    PING               // 心跳/保活
};


// ==================== 1. ROS Bridge (TCP通信) ====================

/**
 * ROSBridge - 与小车ROS系统的TCP通信桥梁
 * 
 * 功能：
 *  - 异步TCP连接管理
 *  - JSON协议编解码
 *  - 自动重连机制
 *  - 心跳保活
 *  - 线程安全消息队列
 */
class ROSBridge {
public:
    using StatusCallback = std::function<void(const SystemStatus&)>;
    using OdomCallback = std::function<void(const OdometryData&)>;
    using RailCallback = std::function<void(const RailStatus&)>;
    using ConnectionCallback = std::function<void(bool)>;  // true=connected, false=disconnected
    using LogCallback = std::function<void(const std::string&)>;

    ROSBridge();
    ~ROSBridge();
    
    // ====== 连接管理 ======
    
    /**
     * 连接到小车ROS服务
     * @param ip 小车IP地址 (如 "192.168.1.100")
     * @param port 端口号 (默认1955)
     * @return 是否成功启动连接
     */
    bool connect(const std::string& ip, int port = 1955);
    
    /** 断开连接 */
    void disconnect();
    
    /** 是否已连接 */
    bool isConnected() const { return connected_.load(); }
    
    /** 获取目标IP */
    const std::string& getTargetIP() const { return target_ip_; }
    
    // ====== 命令发送 ======
    
    /** 发送速度指令 */
    void sendCmdVel(float linear_x, float angular_z);
    
    /** 切换模式 (STANDBY/AUTO/MANUAL) */
    void sendModeChange(const std::string& mode);
    
    /** 发送急停指令 */
    void sendEmergencyStop();
    
    /** 发送云台控制 */
    void sendGimbalCmd(float yaw_deg, float pitch_deg);
    
    /** 发送LED开关 */
    void sendLEDToggle(bool on);
    
    // ====== 回调注册 ======
    
    void onSystemStatus(StatusCallback cb) { status_cb_ = cb; }
    void onOdometry(OdomCallback cb) { odom_cb_ = cb; }
    void onRailStatus(RailCallback cb) { rail_cb_ = cb; }
    void onConnectionChanged(ConnectionCallback cb) { conn_cb_ = cb; }
    void onLog(LogCallback cb) { log_cb_ = cb; }
    
private:
    // 内部方法
    void initSocket();
    void cleanupSocket();
    void receiveLoop();
    void sendMessage(const json& msg);
    void parseMessage(const std::string& raw_msg);
    void handlePing();
    void reconnectLoop();  // 自动重连线程
    
    // Socket相关
    socket_t socket_fd_;
    struct sockaddr_in server_addr_;
    std::string target_ip_;
    int target_port_;
    
    // 状态
    std::atomic<bool> connected_{false};
    std::atomic<bool> should_run_{false};
    
    // 线程
    std::thread recv_thread_;
    std::thread reconnect_thread_;
    
    // 回调
    StatusCallback status_cb_;
    OdomCallback odom_cb_;
    RailCallback rail_cb_;
    ConnectionCallback conn_cb_;
    LogCallback log_cb_;
    
    // 同步
    std::mutex send_mutex_;
    std::mutex recv_mutex_;
    
    // 心跳
    std::atomic<double> last_ping_time_{0.0};
    std::thread ping_thread_;
    static constexpr double PING_INTERVAL = 2.0;  // 秒
    static constexpr double CONNECTION_TIMEOUT = 5.0;  // 秒
    
    // 重连配置
    std::atomic<bool> auto_reconnect_{true};
    static constexpr int RECONNECT_INTERVAL_MS = 3000;
};

// ==================== 2. Virtual Joystick (虚拟摇杆) ====================

/**
 * VirtualJoystick - 屏幕虚拟摇杆控件
 * 
 * 特点：
 *  - 支持鼠标拖拽和触摸操作
 *  - 输出归一化值 [-1, 1]
 *  - 松手自动回中
 *  - 可视化反馈
 */
class VirtualJoystick {
public:
    using ValueChangedCallback = std::function<void(float linear_x, float angular_z)>;
    
    VirtualJoystick();
    ~VirtualJoystick() = default;
    
    // ====== 几何配置 ======
    
    /** 设置位置和大小 */
    void setBounds(float x, float y, float size);
    
    /** 获取边界 */
    float getX() const { return x_; }
    float getY() const { return y_; }
    float getSize() const { return size_; }
    
    // ====== 输入处理 ======
    
    bool onMouseDown(float mx, float my);   // 返回是否点中
    void onMouseMove(float mx, float my);
    void onMouseUp(float mx, float my);
    
    // ====== 数据获取 ======
    
    /** 获取线性速度分量 [-1, 1] (前后) */
    float getLinearX() const;
    
    /** 获取角速度分量 [-1, 1] (左右转向) */
    float getAngularZ() const;
    
    /** 是否正在被操作 */
    bool isActive() const { return is_dragging_; }
    
    // ====== 回调 ======
    
    void onValueChanged(ValueChangedCallback cb) { value_changed_cb_ = cb; }
    
    // ====== 渲染 (需在OpenGL上下文中调用) ======
    
    void render();  // 绘制摇杆（背景圆 + knob）
    
private:
    void updateKnobPosition(float mx, float my);
    void clampKnobToCircle();
    void notifyValueChanged();
    
    // 几何
    float x_, y_;          // 左上角位置
    float size_;           // 正方形边长
    float center_x_, center_y_;  // 圆心
    float radius_;         // 活动半径
    float knob_radius_;    // knob半径
    
    // 状态
    float knob_x_, knob_y_;  // knob当前位置(相对center的偏移)
    bool is_dragging_;
    
    // 外观颜色 (RGBA, 0-255)
    struct Color { uint8_t r, g, b, a; };
    Color bg_color_ = {60, 60, 70, 200};        // 背景圆(深灰半透明)
    Color ring_color_ = {100, 100, 110, 255};   // 外环
    Color knob_color_ = {0, 180, 255, 255};     // knob(蓝色)
    Color active_color_ = {0, 220, 100, 255};   // 激活时(绿色)
    
    // 回调
    ValueChangedCallback value_changed_cb_;
    
    // 上次通知的值(避免重复触发)
    float last_notified_linear_x_ = 0;
    float last_notified_angular_z_ = 0;
    static constexpr float NOTIFY_THRESHOLD = 0.02f;  // 最小变化量
};


// ==================== 3. Car Control Panel (控制面板) ====================

/**
 * CarControlPanel - 小车控制主面板
 * 
 * 包含：
 *  - 连接管理 (IP输入 + 连接/断开按钮)
 *  - 虚拟摇杆
 *  - 急停按钮
 *  - 模式切换按钮
 *  - 状态显示区域
 *  - 云台控制滑块
 *  - 日志输出窗口
 */
class CarControlPanel {
public:
    CarControlPanel();
    ~CarControlPanel();
    
    // ====== 生命周期 ======
    
    void init();       // 初始化所有子组件
    void update();     // 每帧更新(刷新数据、处理输入)
    void render();     // 渲染整个面板
    
    // ====== 尺寸与布局 ======
    
    void setBounds(float x, float y, float width, float height);
    float getWidth() const { return width_; }
    float getHeight() const { return height_; }
    
    // ====== 事件处理 ======
    
    bool onMouseDown(float mx, float my);   // 返回是否消费事件
    void onMouseMove(float mx, float my);
    void onMouseUp(float mx, float my);
    void onKeyDown(int key);
    void onKeyUp(int key);
    
    // ====== 数据访问 ======
    
    const SystemStatus& getSystemStatus() const { return current_status_; }
    const OdometryData& getOdometry() const { return current_odom_; }
    const RailStatus& getRailStatus() const { return current_rail_status_; }
    bool isConnected() const { return bridge_.isConnected(); }
    
private:
    // 子组件
    VirtualJoystick joystick_;
    
    // ROS桥接
    ROSBridge bridge_;
    
    // 数据模型
    SystemStatus current_status_;
    OdometryData current_odom_;
    RailStatus current_rail_status_;
    
    // 布局
    float x_, y_, width_, height_;
    
    // UI元素位置(相对坐标，渲染时转换为绝对坐标)
    struct Rect { float x, y, w, h; };
    Rect connect_rect_;       // 连接区域
    Rect status_rect_;        // 状态显示区
    Rect control_rect_;       // 控制区
    Rect log_rect_;           // 日志区
    
    // 连接相关
    std::string input_ip_;
    bool ip_input_focused_;
    
    // 日志
    std::vector<std::string> log_messages_;
    static constexpr size_t MAX_LOG_MESSAGES = 50;
    void addLog(const std::string& msg);
    
    // 内部回调(桥接数据到达时更新本地缓存)
    void setupCallbacks();
};


// ==================== 4. 辅助工具函数 ====================

namespace utils {

/** 角度归一化到 [-π, π] */
inline float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/** 限制值范围 */
inline float clamp(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(val, max_val));
}

/** 线性映射 [in_min,in_max] → [out_min,out_max] */
inline float mapRange(float val, float in_min, float in_max, 
                       float out_min, float out_max) {
    return out_min + (out_max - out_min) * ((val - in_min) / (in_max - in_min));
}

/** 获取当前时间戳(秒) */
inline double getCurrentTime() {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

} // namespace utils


} // namespace car_control

#endif // CAR_CONTROL_MODULE_H

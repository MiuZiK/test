/**
 * ==================================================================================
 * Aviateur 小车控制模块 - 实现文件
 * ==================================================================================
 * 
 * 包含：
 *  - ROSBridge TCP通信实现
 *  - VirtualJoystick 虚拟摇杆实现
 *  - CarControlPanel 控制面板实现
 */

#include "car_control_module.h"
#include <iostream>
#include <sstream>
#include <algorithm>

namespace car_control {

// ==================================================================================
// ROSBridge 实现
// ==================================================================================

ROSBridge::ROSBridge() 
    : socket_fd_(INVALID_SOCKET_VAL)
    , target_port_(1955)
    , connected_(false)
    , should_run_(false) {
    
#ifdef _WIN32
    // 初始化Winsock
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

ROSBridge::~ROSBridge() {
    disconnect();
    
#ifdef _WIN32
    WSACleanup();
#endif
}

bool ROSBridge::connect(const std::string& ip, int port) {
    if (connected_.load()) {
        disconnect();
    }
    
    target_ip_ = ip;
    target_port_ = port;
    
    if (log_cb_) log_cb_("[Bridge] Connecting to " + ip + ":" + std::to_string(port));
    
    // 创建socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == INVALID_SOCKET_VAL) {
        if (log_cb_) log_cb_("[Bridge] ERROR: Failed to create socket");
        return false;
    }
    
    // 设置非阻塞模式 + 超时
#ifdef _WIN32
    u_long mode = 1;  // 非阻塞
    ioctlsocket(socket_fd_, FIONBIO, &mode);
#else
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
#endif
    
    // 配置地址
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
#ifdef _WIN32
    server_addr_.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
#else
    inet_pton(AF_INET, ip.c_str(), &server_addr_.sin_addr);
#endif
    
    // 尝试连接
    int result = ::connect(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
    
    if (result < 0) {
#ifdef _WIN32
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAEINPROGRESS) {
            // 正在连接中，使用select等待
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(socket_fd_, &write_fds);
            
            timeval timeout;
            timeout.tv_sec = 3;   // 3秒超时
            timeout.tv_usec = 0;
            
            int sel = select(0, NULL, &write_fds, NULL, &timeout);
            if (sel > 0) {
                int so_error = 0;
                socklen_t len = sizeof(so_error);
                getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, (char*)&so_error, &len);
                if (so_error == 0) {
                    result = 0;  // 连接成功
                }
            }
        }
#endif
        
        if (result < 0) {
            if (log_cb_) log_cb_("[Bridge] ERROR: Connection failed");
            cleanupSocket();
            return false;
        }
    }
    
    // 恢复为阻塞模式（用于接收线程）
#ifdef _WIN32
    u_long mode = 0;
    ioctlsocket(socket_fd_, FIONBIO, &mode);
#else
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags & ~O_NONBLOCK);
#endif
    
    // 标记已连接
    connected_.store(true);
    should_run_.store(true);
    last_ping_time_.store(utils::getCurrentTime());
    
    // 启动接收线程
    recv_thread_ = std::thread(&ROSBridge::receiveLoop, this);
    
    // 启动心跳线程
    ping_thread_ = std::thread([this]() {
        while (should_run_.load() && connected_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                (int)(PING_INTERVAL * 1000)));
            handlePing();
        }
    });
    
    // 启动重连监控线程
    reconnect_thread_ = std::thread(&ROSBridge::reconnectLoop, this);
    
    if (log_cb_) log_cb_("[Bridge] ✓ Connected successfully!");
    if (conn_cb_) conn_cb_(true);
    
    return true;
}

void ROSBridge::disconnect() {
    should_run_.store(false);
    connected_.store(false);
    
    if (conn_cb_) conn_cb_(false);
    
    // 等待线程结束
    if (recv_thread_.joinable()) recv_thread_.join();
    if (ping_thread_.joinable()) ping_thread_.join();
    if (reconnect_thread_.joinable()) reconnect_thread_.join();
    
    cleanupSocket();
    
    if (log_cb_) log_cb_("[Bridge] Disconnected");
}

void ROSBridge::cleanupSocket() {
    if (socket_fd_ != INVALID_SOCKET_VAL) {
        CLOSE_SOCKET(socket_fd_);
        socket_fd_ = INVALID_SOCKET_VAL;
    }
}

void ROSBridge::receiveLoop() {
    char buffer[4096];
    
    while (should_run_.load() && connected_.load()) {
        // 接收数据
        ssize_t bytes = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
        
        if (bytes <= 0) {
            // 连接断开或错误
            if (bytes == 0) {
                if (log_cb_) log_cb_("[Bridge] Server closed connection");
            } else {
#ifdef _WIN32
                if (log_cb_) log_cb_("[Bridge] Recv error: " + std::to_string(WSAGetLastError()));
#else
                if (log_cb_) log_cb_("[Bridge] Recv error: " + std::to_string(errno));
#endif
            }
            connected_.store(false);
            if (conn_cb_) conn_cb_(false);
            break;
        }
        
        buffer[bytes] = '\0';
        std::string msg(buffer);
        
        // 解析JSON消息
        parseMessage(msg);
    }
}

void ROSBridge::parseMessage(const std::string& raw_msg) {
    try {
        json j = json::parse(raw_msg);
        
        std::string type = j.value("type", "");
        
        if (type == "system_status") {
            SystemStatus status = SystemStatus::from_json(j["data"]);
            current_status_ = status;  // 缓存
            if (status_cb_) status_cb_(status);
        }
        else if (type == "odometry") {
            OdometryData odom = OdometryData::from_json(j["data"]);
            current_odom_ = odom;
            if (odom_cb_) odom_cb_(odom);
        }
        else if (type == "rail_status") {
            RailStatus rail = RailStatus::from_json(j["data"]);
            current_rail_status_ = rail;
            if (rail_cb_) rail_cb_(rail);
        }
        else if (type == "pong") {
            last_ping_time_.store(utils::getCurrentTime());
            // 心跳响应，连接正常
        }
        else if (type == "error") {
            std::string err_msg = j.value("message", "Unknown error");
            if (log_cb_) log_cb_("[Bridge] Error from server: " + err_msg);
        }
        else {
            if (log_cb_) log_cb_("[Bridge] Unknown message type: " + type);
        }
        
    } catch (const json::exception& e) {
        if (log_cb_) log_cb_("[Bridge] JSON parse error: " + std::string(e.what()));
    }
}

void ROSBridge::sendMessage(const json& msg) {
    if (!connected_.load()) return;
    
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    std::string serialized = msg.dump();
    
    // 发送长度前缀(4字节大端序)
    uint32_t len = static_cast<uint32_t>(serialized.size());
    len = htonl(len);
    
    send(socket_fd_, reinterpret_cast<const char*>(&len), 4, 0);
    send(socket_fd_, serialized.c_str(), serialized.size(), 0);
}

void ROSBridge::sendCmdVel(float linear_x, float angular_z) {
    json msg = {
        {"type", "cmd_vel"},
        {"timestamp", utils::getCurrentTime()},
        {"data", {
            {"linear_x", linear_x},
            {"angular_z", angular_z}
        }}
    };
    sendMessage(msg);
    
    if (log_cb_) {
        std::ostringstream oss;
        oss << "[CMD] cmd_vel: linear=" << linear_x << " angular=" << angular_z;
        log_cb_(oss.str());
    }
}

void ROSBridge::sendModeChange(const std::string& mode) {
    json msg = {
        {"type", "mode_change"},
        {"data", {{"mode", mode}}}
    };
    sendMessage(msg);
    
    if (log_cb_) log_cb_("[CMD] Mode change: " + mode);
}

void ROSBridge::sendEmergencyStop() {
    json msg = {
        {"type", "emergency_stop"},
        {"data", json::object()}
    };
    sendMessage(msg);
    
    if (log_cb_) log_cb_("[CMD] ⛑ EMERGENCY STOP!");
}

void ROSBridge::sendGimbalCmd(float yaw_deg, float pitch_deg) {
    json msg = {
        {"type", "gimbal_cmd"},
        {"data", {
            {"yaw", yaw_deg},
            {"pitch", pitch_deg}
        }}
    };
    sendMessage(msg);
    
    if (log_cb_) {
        std::ostringstream oss;
        oss << "[CMD] Gimbal: yaw=" << yaw_deg << " pitch=" << pitch_deg;
        log_cb_(oss.str());
    }
}

void ROSBridge::sendLEDToggle(bool on) {
    json msg = {
        {"type", "led_toggle"},
        {"data", {{"on", on}}}
    };
    sendMessage(msg);
    
    if (log_cb_) log_cb_(std::string("[CMD] LED ") + (on ? "ON" : "OFF"));
}

void ROSBridge::handlePing() {
    if (!connected_.load()) return;
    
    double now = utils::getCurrentTime();
    
    // 检查是否超时
    if (now - last_ping_time_.load() > CONNECTION_TIMEOUT) {
        if (log_cb_) log_cb_("[Bridge] ⚠ Connection timeout!");
        connected_.store(false);
        if (conn_cb_) conn_cb_(false);
        return;
    }
    
    // 发送ping
    json msg = {
        {"type", "ping"},
        {"timestamp", now}
    };
    sendMessage(msg);
}

void ROSBridge::reconnectLoop() {
    while (should_run_.load()) {
        if (!connected_.load() && auto_reconnect_.load()) {
            if (log_cb_) log_cb_("[Bridge] Attempting to reconnect...");
            
            std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_INTERVAL_MS));
            
            if (!target_ip_.empty()) {
                connect(target_ip_, target_port_);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}


// ==================================================================================
// VirtualJoystick 实现
// ==================================================================================

VirtualJoystick::VirtualJoystick()
    : x_(0), y_(0), size_(200)
    , center_x_(100), center_y_(100)
    , radius_(80), knob_radius_(25)
    , knob_x_(0), knob_y_(0)
    , is_dragging_(false) {
}

void VirtualJoystick::setBounds(float x, float y, float size) {
    x_ = x;
    y_ = y;
    size_ = size;
    center_x_ = x + size / 2.0f;
    center_y_ = y + size / 2.0f;
    radius_ = size * 0.4f;
    knob_radius_ = size * 0.12f;
}

bool VirtualJoystick::onMouseDown(float mx, float my) {
    // 检查是否点击在摇杆范围内
    float dx = mx - center_x_;
    float dy = my - center_y_;
    float dist = sqrt(dx*dx + dy*dy);
    
    if (dist <= radius_ + knob_radius_) {
        is_dragging_ = true;
        updateKnobPosition(mx, my);
        return true;
    }
    return false;
}

void VirtualJoystick::onMouseMove(float mx, float my) {
    if (is_dragging_) {
        updateKnobPosition(mx, my);
    }
}

void VirtualJoystick::onMouseUp(float mx, float my) {
    if (is_dragging_) {
        is_dragging_ = false;
        // 自动回中
        knob_x_ = 0;
        knob_y_ = 0;
        notifyValueChanged();  // 发送零速度
    }
}

float VirtualJoystick::getLinearX() const {
    // Y轴向上为正，但小车前进方向是Y轴负方向
    return -knob_y_ / radius_;  // 归一化到[-1, 1]
}

float VirtualJoystick::getAngularZ() const {
    // X轴向右为正，右转为正角速度
    return knob_x_ / radius_;  // 归一化到[-1, 1]
}

void VirtualJoystick::updateKnobPosition(float mx, float my) {
    knob_x_ = mx - center_x_;
    knob_y_ = my - center_y_;
    
    clampKnobToCircle();
    notifyValueChanged();
}

void VirtualJoystick::clampKnobToCircle() {
    float dist = sqrt(knob_x_*knob_x_ + knob_y_*knob_y_);
    
    if (dist > radius_) {
        // 限制在圆内
        float scale = radius_ / dist;
        knob_x_ *= scale;
        knob_y_ *= scale;
    }
    
    // 死区处理(中心小范围不触发)
    static constexpr float DEADZONE = 5.0f;
    if (fabs(knob_x_) < DEADZONE && fabs(knob_y_) < DEADZONE) {
        knob_x_ = 0;
        knob_y_ = 0;
    }
}

void VirtualJoystick::notifyValueChanged() {
    if (!value_changed_cb_) return;
    
    float lx = getLinearX();
    float az = getAngularZ();
    
    // 只有变化超过阈值才通知
    if (fabs(lx - last_notified_linear_x_) > NOTIFY_THRESHOLD ||
        fabs(az - last_notified_angular_z_) > NOTIFY_THRESHOLD) {
        
        value_changed_cb_(lx, az);
        last_notified_linear_x_ = lx;
        last_notified_angular_z_ = az;
    }
}

void VirtualJoystick::render() {
    // 注意：此函数需要在OpenGL上下文中调用
    // 这里提供伪代码，实际实现需根据Aviateur的Revector框架调整
    
    /*
    // 1. 绘制背景圆(半透明深灰)
    glColor4ub(bg_color_.r, bg_color_.g, bg_color_.b, bg_color_.a);
    drawFilledCircle(center_x_, center_y_, radius_);
    
    // 2. 绘制外环
    glColor4ub(ring_color_.r, ring_color_.g, ring_color_.b, ring_color_.a);
    drawCircle(center_x_, center_y_, radius_, 3.0f);
    
    // 3. 绘制十字参考线
    glColor4ub(100, 100, 100, 150);
    drawLine(center_x_ - radius_*0.6f, center_y_, 
             center_x_ + radius_*0.6f, center_y_, 1.0f);  // X轴
    drawLine(center_x_, center_y_ - radius_*0.6f, 
             center_x_, center_y_ + radius_*0.6f, 1.0f);  // Y轴
    
    // 4. 绘制knob
    Color& c = is_dragging_ ? active_color_ : knob_color_;
    glColor4ub(c.r, c.g, c.b, c.a);
    drawFilledCircle(center_x_ + knob_x_, center_y_ + knob_y_, knob_radius_);
    
    // 5. 绘制knob高光效果
    glColor4ub(255, 255, 255, 100);
    drawFilledCircle(center_x_ + knob_x_ - knob_radius_*0.3f, 
                     center_y_ + knob_y_ - knob_radius_*0.3f, 
                     knob_radius_*0.3f);
    */
}


// ==================================================================================
// CarControlPanel 实现
// ==================================================================================

CarControlPanel::CarControlPanel()
    : x_(0), y_(0), width_(300), height_(600)
    , ip_input_focused_(false) {
    
    setupCallbacks();
}

CarControlPanel::~CarControlPanel() {
    bridge_.disconnect();
}

void CarControlPanel::init() {
    // 初始化摇杆位置
    joystick_.setBounds(x_ + 50, y_ + 200, 200);
    
    // 布局各区域
    connect_rect_ = {x_ + 10, y_ + 10, width_ - 20, 80};
    status_rect_ = {x_ + 10, y_ + 100, width_ - 20, 90};
    control_rect_ = {x_ + 10, y_ + 200, width_ - 20, 250};
    log_rect_ = {x_ + 10, y_ + 460, width_ - 20, 130};
    
    input_ip_ = "192.168.1.100";  // 默认IP
    
    addLog("Car Control Panel initialized");
}

void CarControlPanel::setBounds(float x, float y, float w, float h) {
    x_ = x;
    y_ = y;
    width_ = w;
    height_ = h;
    init();  // 重新计算子区域布局
}

void CarControlPanel::setupCallbacks() {
    // 注册ROS Bridge回调
    bridge_.onSystemStatus([this](const SystemStatus& s) {
        current_status_ = s;
    });
    
    bridge_.onOdometry([this](const OdometryData& o) {
        current_odom_ = o;
    });
    
    bridge_.onRailStatus([this](const RailStatus& r) {
        current_rail_status_ = r;
    });
    
    bridge_.onConnectionChanged([this](bool connected) {
        addLog(std::string("Connection ") + (connected ? "established" : "lost"));
    });
    
    bridge_.onLog([this](const std::string& msg) {
        addLog(msg);
    });
    
    // 注册摇杆回调
    joystick_.onValueChanged([this](float lx, float az) {
        if (bridge_.isConnected()) {
            // 映射到实际速度范围
            float max_linear = 0.33f;
            float max_angular = 0.5f;
            
            bridge_.sendCmdVel(lx * max_linear, az * max_angular);
        }
    });
}

void CarControlPanel::update() {
    // 定期刷新显示数据(由回调驱动，此处可添加额外逻辑)
}

void CarControlPanel::render() {
    // 注意：完整渲染代码需要集成Aviateur的Revector UI框架
    // 此处提供布局伪代码
    
    /*
    // ====== 背景 ======
    drawRect(x_, y_, width_, height_, {30, 30, 35, 255});
    
    // ====== 标题栏 ======
    drawText(x_ + 10, y_ + 25, "🚗 Undercarriage Control", {200, 200, 200, 255}, 16);
    
    // ====== 连接区域 ======
    drawRect(connect_rect_.x, connect_rect_.y, connect_rect_.w, connect_rect_.h, 
             {45, 45, 50, 255});
    drawText(connect_rect_.x + 10, connect_rect_.y + 15, "Connection:", {180, 180, 180, 255}, 14);
    
    // IP输入框
    bool is_connected = bridge_.isConnected();
    drawTextInput(connect_rect_.x + 10, connect_rect_.y + 35, 
                  connect_rect_.w - 120, 25,
                  input_ip_, ip_input_focused_,
                  is_connected ? {60, 60, 70, 255} : {255, 255, 255, 255});
    
    // 连接/断开按钮
    if (is_connected) {
        drawButton(connect_rect_.x + connect_rect_.w - 100, connect_rect_.y + 35,
                  90, 25, "Disconnect", {180, 60, 60, 255});  // 红色
    } else {
        drawButton(connect_rect_.x + connect_rect_.w - 100, connect_rect_.y + 35,
                  90, 25, "Connect", {60, 140, 60, 255});     // 绿色
    }
    
    // 状态指示灯
    drawCircle(connect_rect_.x + connect_rect_.w - 15, connect_rect_.y + 47, 8,
              is_connected ? Color{0, 220, 0, 255} : Color{180, 60, 60, 255});
    
    // ====== 状态面板 ======
    drawRect(status_rect_.x, status_rect_.y, status_rect_.w, status_rect_.h,
             {40, 40, 45, 255});
    drawText(status_rect_.x + 10, status_rect_.y + 10, "📊 Status", {160, 160, 170, 255}, 14);
    
    // 模式
    std::string mode_str = "Mode: " + current_status_.mode;
    drawText(status_rect_.x + 10, status_rect_.y + 30, mode_str, {220, 220, 220, 255}, 13);
    
    // 行驶距离
    char dist_buf[64];
    snprintf(dist_buf, sizeof(dist_buf), "Distance: %.1fm", current_odom_.traveled_distance);
    drawText(status_rect_.x + 10, status_rect_.y + 48, dist_buf, {180, 180, 180, 255}, 12);
    
    // 位置
    char pos_buf[64];
    snprintf(pos_buf, sizeof(pos_buf), "Pos: (%.1f, %.1f)", current_odom_.x, current_odom_.y);
    drawText(status_rect_.x + 10, status_rect_.y + 63, pos_buf, {160, 160, 160, 255}, 12);
    
    // 电池
    char bat_buf[64];
    snprintf(bat_buf, sizeof(bat_buf), "Battery: %.1fV", current_status_.battery_voltage);
    Color bat_color = current_status_.battery_voltage > 11.0 ? 
                      Color{60, 180, 60, 255} : Color{220, 160, 60, 255};
    drawText(status_rect_.x + 110, status_rect_.y + 63, bat_buf, bat_color, 12);
    
    // ====== 控制区域 ======
    drawRect(control_rect_.x, control_rect_.y, control_rect_.w, control_rect_.h,
             {38, 38, 43, 255});
    drawText(control_rect_.x + 10, control_rect_.y + 10, "🎮 Control", {160, 160, 170, 255}, 14);
    
    // 渲染虚拟摇杆
    joystick_.render();
    
    // 急停按钮 (大号红色)
    drawButton(control_rect_.x + 50, control_rect_.y + 220, 100, 35,
              "⛑ EMERGENCY STOP", {200, 50, 50, 255}, 16);
    
    // 模式切换按钮
    const char* mode_btn_text = current_status_.mode == "AUTO" ? "MODE: AUTO" : 
                               current_status_.mode == "MANUAL" ? "MODE: MANUAL" : "MODE: STANDBY";
    drawButton(control_rect_.x + 160, control_rect_.y + 220, 90, 35,
              mode_btn_text, {70, 70, 80, 255}, 11);
    
    // ====== 日志区域 ======
    drawRect(log_rect_.x, log_rect_.y, log_rect_.w, log_rect_.h,
             {35, 35, 40, 255});
    drawText(log_rect_.x + 10, log_rect_.y + 5, "📝 Log", {140, 140, 150, 255}, 12);
    
    // 显示最近的日志消息
    int y_offset = 22;
    size_t start_idx = log_messages_.size() > 8 ? log_messages_.size() - 8 : 0;
    for (size_t i = start_idx; i < log_messages_.size(); i++) {
        drawText(log_rect_.x + 5, log_rect_.y + y_offset, 
                 log_messages_[i], {150, 150, 150, 255}, 10);
        y_offset += 14;
    }
    */
}

bool CarControlPanel::onMouseDown(float mx, float my) {
    // 检查各UI元素
    
    // IP输入框
    if (mx >= connect_rect_.x + 10 && mx <= connect_rect_.x + connect_rect_.w - 120 &&
        my >= connect_rect_.y + 35 && my <= connect_rect_.y + 60) {
        ip_input_focused_ = true;
        return true;
    }
    
    // 连接/断开按钮
    if (mx >= connect_rect_.x + connect_rect_.w - 100 && 
        mx <= connect_rect_.x + connect_rect_.w - 10 &&
        my >= connect_rect_.y + 35 && my <= connect_rect_.y + 60) {
        onConnectClicked();
        return true;
    }
    
    // 急停按钮
    if (mx >= control_rect_.x + 50 && mx <= control_rect_.x + 150 &&
        my >= control_rect_.y + 220 && my <= control_rect_.y + 255) {
        onStopClicked();
        return true;
    }
    
    // 模式按钮
    if (mx >= control_rect_.x + 160 && mx <= control_rect_.x + 250 &&
        my >= control_rect_.y + 220 && my <= control_rect_.y + 255) {
        // 循环切换模式
        std::string new_mode;
        if (current_status_.mode == "STANDBY") new_mode = "AUTO";
        else if (current_status_.mode == "AUTO") new_mode = "MANUAL";
        else new_mode = "STANDBY";
        onModeChanged(new_mode);
        return true;
    }
    
    // 虚拟摇杆
    if (joystick_.onMouseDown(mx, my)) {
        return true;
    }
    
    return false;
}

void CarControlPanel::onMouseMove(float mx, float my) {
    joystick_.onMouseMove(mx, my);
}

void CarControlPanel::onMouseUp(float mx, float my) {
    ip_input_focused_ = false;
    joystick_.onMouseUp(mx, my);
}

void CarControlPanel::onKeyDown(int key) {
    if (ip_input_focused_) {
        // 处理IP输入
        if (key >= '0' && key <= '9' || key == '.') {
            input_ip_ += (char)key;
        } else if (key == '\b' && !input_ip_.empty()) {
            input_ip_.pop_back();
        }
    }
    
    // 键盘快捷键
    switch (key) {
        case 'W': case 'w':
            bridge_.sendCmdVel(0.33f, 0.0f);
            break;
        case 'S': case 's':
            bridge_.sendCmdVel(-0.33f, 0.0f);
            break;
        case 'A': case 'a':
            bridge_.sendCmdVel(0.0f, 0.5f);
            break;
        case 'D': case 'd':
            bridge_.sendCmdVel(0.0f, -0.5f);
            break;
        case ' ':  // 空格键急停
            onStopClicked();
            break;
    }
}

void CarControlPanel::onKeyUp(int key) {
    // 松开方向键时停止
    if ((key == 'W' || key == 'w' || key == 'S' || key == 's' ||
         key == 'A' || key == 'a' || key == 'D' || key == 'd')) {
        if (bridge_.isConnected()) {
            bridge_.sendCmdVel(0.0f, 0.0f);
        }
    }
}

void CarControlPanel::onConnectClicked() {
    if (bridge_.isConnected()) {
        bridge_.disconnect();
    } else {
        bridge_.connect(input_ip_);
    }
}

void CarControlPanel::onStopClicked() {
    bridge_.sendEmergencyStop();
}

void CarControlPanel::onModeChanged(const std::string& mode) {
    bridge_.sendModeChange(mode);
    current_status_.mode = mode;
}

void CarControlPanel::addLog(const std::string& msg) {
    log_messages_.push_back(msg);
    
    // 限制日志数量
    while (log_messages_.size() > MAX_LOG_MESSAGES) {
        log_messages_.erase(log_messages_.begin());
    }
}


} // namespace car_control

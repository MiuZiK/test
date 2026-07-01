#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>

// System state enumeration
enum class ControlState {
    INIT,               // Initialization
    ALIGN_DIRECTION,    // Align to target direction
    AUTO_STRAIGHT,      // Auto straight driving
    WAIT_RETURN,        // Waiting for return trigger
    RETURN_HOME,        // Returning to home position
    TURNING             // Turning to face home
};

// 云台控制参数配置
struct GimbalConfig {
    int roll_channel = 3;     // CH4控制水平旋转（左右推动）
    int pitch_channel = 2;    // CH3控制垂直旋转（前后推动）
    int mode_channel = 5;     // CH6控制启动/停止
    float deadzone = 15.0f;   // 云台摇杆死区
    int min_value = -100;     // 云台控制量最小值
    int max_value = 100;      // 云台控制量最大值
    int mode_low = 1000;      // 模式通道下档阈值
    int mode_high = 2000;     // 模式通道上档阈值
} gimbal_cfg;

// 云台状态枚举
enum GimbalMode {
    STOPPED,   // 停止
    STARTED,   // 启动
    RUNNING    // 运行中
};

// 全局变量
ros::Publisher cmd_vel_pub;
ros::Publisher gimbal_cmd_pub;
ros::Timer gimbal_timer;  // 异步发送云台指令的定时器
int gimbal_send_count = 0;  // 指令发送计数
nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::INIT;
GimbalMode gimbal_mode = STOPPED;
int last_roll = 0;
int last_pitch = 0;
ros::Time last_gimbal_cmd_time;
float gimbal_cmd_interval = 0.05f;

// Control parameters
float linear_x = 0.28f;
float turn_speed = 0.5f;
float kp = 1.5f;
float ki = 0.08f;
float kd = 0.3f;
float feedforward_compensation = 0.04f;
float last_yaw_error = 0.0f;
float integral_error = 0.0f;
ros::Time last_callback_time;
float target_distance = 3.0f;
float target_tolerance = 0.15f;
float near_target_threshold = 0.3f;
float angle_tolerance = 0.1f;
float alignment_tolerance = 0.05f;

// Coordinate parameters
float home_x = 0.0f, home_y = 0.0f;
float home_yaw = 0.0f;
float target_x = 0.0f, target_y = 0.0f;
float target_direction = 0.0f;

// Remote control related
bool remote_override = false;
bool prev_remote_override = false;
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 15.0f;
const float MAX_ANGULAR = 0.5f;

// 定时器回调：异步发送云台启动指令
void gimbalTimerCallback(const ros::TimerEvent& event) {
    if (gimbal_send_count > 0) {
        std_msgs::String cmd_msg;
        cmd_msg.data = "start";
        gimbal_cmd_pub.publish(cmd_msg);
        ROS_INFO("[Gimbal] Async start command sent (attempt %d/2)", 3 - gimbal_send_count);
        gimbal_send_count--;
    } else {
        gimbal_timer.stop();  // 计数为0时停止定时器
    }
}

// Calculate distance between two points
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Calculate the shortest angular distance
float shortestAngularDistance(float from, float to) {
    float diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

// 将摇杆值映射为云台控制量
int mapGimbalValue(int mapped_value, float deadzone) {
    if (fabs(mapped_value) < deadzone) {
        return 0;
    }
    float scale = (mapped_value > 0) ? 
        (gimbal_cfg.max_value / 100.0f) : 
        (gimbal_cfg.min_value / -100.0f);
    return static_cast<int>(round(mapped_value * scale));
}

// 处理云台模式控制（异步发送指令，解决卡顿）
void handleGimbalMode(int raw_value) {
    GimbalMode new_mode = gimbal_mode;
    if (raw_value < gimbal_cfg.mode_low) {
        new_mode = STOPPED;
    } else if (raw_value > gimbal_cfg.mode_high) {
        new_mode = STARTED;
    } else {
        new_mode = RUNNING;
    }

    if (new_mode != gimbal_mode) {
        std_msgs::String cmd_msg;
        if (new_mode == STARTED) {
            cmd_msg.data = "start";
            ROS_INFO("[Gimbal] CH6 up detected, starting async command send");
            // 首次同步发送1次，剩余2次通过定时器异步发送
            gimbal_cmd_pub.publish(cmd_msg);
            ROS_INFO("[Gimbal] Start command sent (attempt 1/3)");
            // 启动定时器：间隔0.2秒，发送剩余2次
            gimbal_send_count = 2;
            gimbal_timer = ros::NodeHandle().createTimer(ros::Duration(0.2), gimbalTimerCallback, false, true);
        } else if (new_mode == STOPPED) {
            cmd_msg.data = "stop";
            ROS_INFO("[Gimbal] CH6 down detected, sending stop command");
            gimbal_cmd_pub.publish(cmd_msg);
            // 停止定时器（如果正在发送）
            if (gimbal_timer.hasPending()) {
                gimbal_timer.stop();
                gimbal_send_count = 0;
            }
        }
        gimbal_mode = new_mode;
    }
}

// 处理云台旋转控制
void handleGimbalControl(const sbus_serial::Sbus::ConstPtr& msg) {
    if (gimbal_mode != RUNNING) return;

    if ((ros::Time::now() - last_gimbal_cmd_time).toSec() < gimbal_cmd_interval) {
        return;
    }

    int roll = mapGimbalValue(msg->mappedChannels[gimbal_cfg.roll_channel], gimbal_cfg.deadzone);
    int pitch = mapGimbalValue(msg->mappedChannels[gimbal_cfg.pitch_channel], gimbal_cfg.deadzone);

    if (roll != last_roll || pitch != last_pitch) {
        std_msgs::String cmd_msg;
        cmd_msg.data = std::to_string(roll) + "," + std::to_string(pitch);
        gimbal_cmd_pub.publish(cmd_msg);
        last_roll = roll;
        last_pitch = pitch;
        last_gimbal_cmd_time = ros::Time::now();
        ROS_DEBUG_THROTTLE(0.5, "[Gimbal Control] Horizontal(CH4): %d, Vertical(CH3): %d", roll, pitch);
    }
}

// Print system state
void printCurrentState() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 1.0) return;
    last_print = ros::Time::now();

    std::string state_str;
    switch (current_state) {
        case ControlState::INIT: state_str = "INIT"; break;
        case ControlState::ALIGN_DIRECTION: state_str = "ALIGN_DIRECTION"; break;
        case ControlState::AUTO_STRAIGHT: state_str = "AUTO_STRAIGHT"; break;
        case ControlState::WAIT_RETURN: state_str = "WAIT_RETURN"; break;
        case ControlState::RETURN_HOME: state_str = "RETURN_HOME"; break;
        case ControlState::TURNING: state_str = "TURNING"; break;
    }

    std::string gimbal_mode_str;
    switch (gimbal_mode) {
        case STOPPED: gimbal_mode_str = "STOPPED"; break;
        case STARTED: gimbal_mode_str = "STARTED"; break;
        case RUNNING: gimbal_mode_str = "RUNNING"; break;
    }

    ROS_INFO("[System State] Car mode: %s | Manual: %s | Gimbal mode: %s", 
             state_str.c_str(), remote_override ? "Yes" : "No", gimbal_mode_str.c_str());
}

// Print remote control information
void printRemoteInfo() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 1.0) return;
    last_print = ros::Time::now();

    std::string ch5_state;
    uint16_t ch5_raw = latest_sbus_data.rawChannels[4];
    if (ch5_raw > 1500)      ch5_state = "Up (trigger return)";
    else if (ch5_raw > 100)  ch5_state = "Middle (normal)";
    else                     ch5_state = "Down (reserved)";

    std::string ch6_state;
    uint16_t ch6_raw = latest_sbus_data.rawChannels[5];
    if (ch6_raw > 2000)      ch6_state = "Up (gimbal start)";
    else if (ch6_raw < 1000) ch6_state = "Down (gimbal stop)";
    else                     ch6_state = "Middle (gimbal control)";

    ROS_INFO("----------------[Remote Control Info]----------------");
    ROS_INFO("CH5(SW A - Return): Raw=%d | State=%s", ch5_raw, ch5_state.c_str());
    ROS_INFO("CH6(SW B - Gimbal): Raw=%d | State=%s", ch6_raw, ch6_state.c_str());
    ROS_INFO("CH1(Steering): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[0],
             latest_sbus_data.mappedChannels[0],
             STICK_DEADZONE);
    ROS_INFO("CH2(Throttle): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[1],
             latest_sbus_data.mappedChannels[1],
             STICK_DEADZONE);
    ROS_INFO("CH3(Gimbal Vertical): Raw=%d | Mapped=%d",
             latest_sbus_data.rawChannels[2],
             latest_sbus_data.mappedChannels[2]);
    ROS_INFO("CH4(Gimbal Horizontal): Raw=%d | Mapped=%d",
             latest_sbus_data.rawChannels[3],
             latest_sbus_data.mappedChannels[3]);
    ROS_INFO("--------------------------------------------");
}

// SBUS callback（修正版，不传递NodeHandle）
void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    printRemoteInfo();
    printCurrentState();

    prev_remote_override = remote_override;

    // 小车手动控制判断
    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    if (ch1_active || ch2_active) {
        remote_override = true;
        
        float angular_input = (ch1_active) ? (msg->mappedChannels[0] / 100.0f) : 0.0f;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] / 100.0f) : 0.0f;
        remote_cmd.linear.x = linear_x * linear_input;

        if (!prev_remote_override) {
            ROS_WARN("[Manual Control] Activated: CH1 or CH2 exceeds deadzone");
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
        }
    } else {
        remote_override = false;
        if (prev_remote_override) {
            ROS_INFO("[Manual Control] Deactivated: Sticks centered");
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0f;
            stop_cmd.angular.z = 0.0f;
            cmd_vel_pub.publish(stop_cmd);
        }
    }

    // 小车返航逻辑
    if (!remote_override) {
        uint16_t ch5_raw = msg->rawChannels[4];
        if (current_state == ControlState::WAIT_RETURN && ch5_raw > 1500) {
            current_state = ControlState::TURNING;
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
            ROS_INFO("✅ CH5 up triggers return! Raw value: %d", ch5_raw);
        } else if ((current_state == ControlState::RETURN_HOME || 
                   current_state == ControlState::TURNING) && ch5_raw <= 100) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ CH5 down cancels return! Raw value: %d", ch5_raw);
        }
    }

    // 云台控制逻辑
    handleGimbalMode(msg->rawChannels[gimbal_cfg.mode_channel]);
    handleGimbalControl(msg);
}

// Odometry callback
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
    has_odom = true;
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_callback_time).toSec();
    last_callback_time = current_time;
    if (dt <= 0.0) dt = 0.01;

    float current_x = msg->pose.pose.position.x;
    float current_y = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, current_yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);

    if (current_state == ControlState::INIT) {
        home_x = current_x;
        home_y = current_y;
        home_yaw = current_yaw;
        target_direction = home_yaw;
        target_x = home_x + target_distance * cos(target_direction);
        target_y = home_y + target_distance * sin(target_direction);
        current_state = ControlState::ALIGN_DIRECTION;
        ROS_INFO("Initialization complete! Home position: (%.2f, %.2f)", home_x, home_y);
        return;
    }

    if (remote_override) {
        cmd_vel_pub.publish(remote_cmd);
        ROS_INFO_THROTTLE(1, "[Manual Command] Linear: %.2f, Angular: %.2f",
                         remote_cmd.linear.x, remote_cmd.angular.z);
        return;
    }

    float current_target_x, current_target_y;
    if (current_state == ControlState::AUTO_STRAIGHT || 
        current_state == ControlState::ALIGN_DIRECTION) {
        current_target_x = target_x;
        current_target_y = target_y;
    } else {
        current_target_x = home_x;
        current_target_y = home_y;
    }

    float remaining_dist = calculateDistance(current_x, current_y, current_target_x, current_target_y);

    if (remaining_dist < target_tolerance) {
        if (current_state == ControlState::AUTO_STRAIGHT || 
            current_state == ControlState::ALIGN_DIRECTION) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Reached target! Waiting for CH5 up to return");
        } else if (current_state == ControlState::RETURN_HOME) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Returned to home!");
        }
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0f;
        stop_cmd.angular.z = 0.0f;
        cmd_vel_pub.publish(stop_cmd);
        return;
    }

    if (current_state == ControlState::ALIGN_DIRECTION) {
        float angle_error = shortestAngularDistance(current_yaw, target_direction);
        if (fabs(angle_error) < alignment_tolerance) {
            current_state = ControlState::AUTO_STRAIGHT;
            ROS_INFO("✅ Direction aligned! Starting straight movement");
            return;
        }
        geometry_msgs::Twist align_cmd;
        align_cmd.linear.x = 0.0f;
        align_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(align_cmd);
        ROS_INFO_THROTTLE(0.5, "[Aligning] Angle error: %.2f rad", angle_error);
        return;
    }

    if (current_state == ControlState::TURNING) {
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        float home_direction = atan2(home_y - current_y, home_x - current_x);
        float angle_error = shortestAngularDistance(current_yaw, home_direction);
        if (fabs(angle_error) < angle_tolerance) {
            current_state = ControlState::RETURN_HOME;
            ROS_INFO("✅ Facing home direction, starting to move");
            return;
        }
        geometry_msgs::Twist turn_cmd;
        turn_cmd.linear.x = 0.0f;
        turn_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(turn_cmd);
        ROS_INFO_THROTTLE(0.5, "[Turning] Angle error: %.2f rad", angle_error);
        return;
    }

    if (current_state == ControlState::AUTO_STRAIGHT || current_state == ControlState::RETURN_HOME) {
        float current_kp = kp;
        if (remaining_dist < near_target_threshold) {
            current_kp *= remaining_dist / near_target_threshold;
        }

        float target_yaw = atan2(current_target_y - current_y, current_target_x - current_x);
        float yaw_error = shortestAngularDistance(current_yaw, target_yaw);
        float yaw_error_rate = (yaw_error - last_yaw_error) / dt;
        last_yaw_error = yaw_error;

        if (fabs(yaw_error) < 0.05f) {
        } else if (yaw_error * last_yaw_error < 0) {
            integral_error = 0.0f;
        } else {
            integral_error += yaw_error * dt;
            integral_error = std::max(std::min(integral_error, 1.0f), -1.0f);
        }

        float angular_z = current_kp * yaw_error + ki * integral_error + kd * yaw_error_rate + feedforward_compensation;
        angular_z = std::max(std::min(angular_z, MAX_ANGULAR), -MAX_ANGULAR);

        float linear_speed = linear_x;
        if (remaining_dist < near_target_threshold) {
            linear_speed *= pow(remaining_dist / near_target_threshold, 2);
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed;
        cmd.angular.z = angular_z;
        cmd_vel_pub.publish(cmd);

        if (current_state == ControlState::AUTO_STRAIGHT) {
            ROS_INFO_THROTTLE(1, "[Auto Driving] Remaining: %.2fm | Yaw error: %.2f",
                             remaining_dist, yaw_error);
        } else if (current_state == ControlState::RETURN_HOME) {
            ROS_INFO_THROTTLE(1, "[Returning Home] Remaining: %.2fm | Yaw error: %.2f",
                             remaining_dist, yaw_error);
        }
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "integrated_car_gimbal_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 小车参数配置
    pnh.param<float>("linear_x", linear_x, 0.28f);
    pnh.param<float>("turn_speed", turn_speed, 0.5f);
    pnh.param<float>("target_distance", target_distance, 3.0f);
    pnh.param<float>("kp", kp, 1.5f);
    pnh.param<float>("ki", ki, 0.08f);
    pnh.param<float>("kd", kd, 0.3f);
    pnh.param<float>("feedforward_compensation", feedforward_compensation, 0.04f);

    // 云台参数配置
    pnh.param<int>("gimbal_roll_channel", gimbal_cfg.roll_channel, 3);
    pnh.param<int>("gimbal_pitch_channel", gimbal_cfg.pitch_channel, 2);
    pnh.param<int>("gimbal_mode_channel", gimbal_cfg.mode_channel, 5);
    pnh.param<float>("gimbal_deadzone", gimbal_cfg.deadzone, 15.0f);

    last_callback_time = ros::Time::now();
    last_gimbal_cmd_time = ros::Time::now();

    // 订阅与发布器初始化
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    gimbal_cmd_pub = nh.advertise<std_msgs::String>("/ui_xy", 10);

    // 节点启动时主动发送start指令
    std_msgs::String init_start_msg;
    init_start_msg.data = "start";
    gimbal_cmd_pub.publish(init_start_msg);
    ROS_INFO("[Gimbal Init] Sent initial start command on node launch");
    ros::Duration(0.5).sleep();
    gimbal_cmd_pub.publish(init_start_msg);
    ROS_INFO("[Gimbal Init] Resent initial start command");

    ROS_INFO("Integrated car & gimbal controller started successfully!");
    ROS_INFO("Gimbal control: CH%d(horizontal), CH%d(vertical), CH%d(mode)",
             gimbal_cfg.roll_channel + 1,
             gimbal_cfg.pitch_channel + 1,
             gimbal_cfg.mode_channel + 1);

    ros::spin();
    return 0;
}
//485..9.12

//398hang straight+re home
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>

// System state enumeration
enum class ControlState {
    INIT,               // Initialization
    ALIGN_DIRECTION,    // Align to target direction
    AUTO_STRAIGHT,      // Auto straight driving
    WAIT_RETURN,        // Waiting for return trigger
    RETURN_HOME,        // Returning to home position
    TURNING             // Turning to face home
};

// Global variables
ros::Publisher cmd_vel_pub;
nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::INIT;

// Control parameters (optimized for tracked vehicle)
float linear_x = 0.28f;            // Driving speed
float turn_speed = 0.5f;           // Turning speed
float kp = 1.5f;                   // Proportional gain
float ki = 0.08f;                  // Integral gain
float kd = 0.3f;                   // Derivative gain
float feedforward_compensation = 0.04f;  // 前馈补偿
float last_yaw_error = 0.0f;
float integral_error = 0.0f;       // 积分误差累积
ros::Time last_callback_time;
float target_distance = 3.0f;
float target_tolerance = 0.15f;
float near_target_threshold = 0.3f;  // Deceleration threshold near target 
float angle_tolerance = 0.1f;        // Angle tolerance for turning (rad)
float alignment_tolerance = 0.05f;   // Alignment tolerance

// Coordinate parameters
float home_x = 0.0f, home_y = 0.0f;
float home_yaw = 0.0f;
float target_x = 0.0f, target_y = 0.0f;
float target_direction = 0.0f;      // Target direction in global frame

// Remote control related (added deadzone parameter)
bool remote_override = false;
bool prev_remote_override = false;  // 添加前一次手动控制状态
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 15.0f;  // 9.10pm new!修改死区值为15（对应[-100,100]范围）
const float MAX_ANGULAR = 0.5f;      // Maximum angular velocity

// Calculate distance between two points
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Calculate the shortest angular distance
float shortestAngularDistance(float from, float to) {
    float diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

// Coordinate transformation (local to global)
void transformLocalToGlobal(float local_x, float local_y, 
                           float& global_x, float& global_y) {
    global_x = home_x + local_x * cos(home_yaw) - local_y * sin(home_yaw);
    global_y = home_y + local_x * sin(home_yaw) + local_y * cos(home_yaw);
}

// Print system state
void printCurrentState() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 1.0) return;
    last_print = ros::Time::now();

    std::string state_str;
    switch (current_state) {
        case ControlState::INIT: state_str = "INIT"; break;
        case ControlState::ALIGN_DIRECTION: state_str = "ALIGN_DIRECTION"; break;
        case ControlState::AUTO_STRAIGHT: state_str = "AUTO_STRAIGHT"; break;
        case ControlState::WAIT_RETURN: state_str = "WAIT_RETURN"; break;
        case ControlState::RETURN_HOME: state_str = "RETURN_HOME"; break;
        case ControlState::TURNING: state_str = "TURNING"; break;
    }
    ROS_INFO("[System State] Current mode: %s | Manual control: %s", 
             state_str.c_str(), remote_override ? "Yes" : "No");
}

// Print remote control information
void printRemoteInfo() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 1.0) return;
    last_print = ros::Time::now();

    // CH5(SW A) state
    std::string ch5_state;
    uint16_t ch5_raw = latest_sbus_data.rawChannels[4];
    if (ch5_raw > 1500)      ch5_state = "Up (trigger return)";
    else if (ch5_raw > 100)  ch5_state = "Middle (normal)";
    else                     ch5_state = "Down (reserved)";

    // CH6(SW B) state
    std::string ch6_state;
    uint16_t ch6_raw = latest_sbus_data.rawChannels[5];
    if (ch6_raw > 1500)      ch6_state = "Up (100)";
    else if (ch6_raw > 100)  ch6_state = "Middle (0)";
    else                     ch6_state = "Down";

    ROS_INFO("----------------[Remote Control Info]----------------");
    ROS_INFO("CH5(SW A): Raw=%d | State=%s", ch5_raw, ch5_state.c_str());
    ROS_INFO("CH6(SW B): Raw=%d | State=%s", ch6_raw, ch6_state.c_str());
    ROS_INFO("CH1(Steering): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[0],
             latest_sbus_data.mappedChannels[0],
             STICK_DEADZONE);
    ROS_INFO("CH2(Throttle): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[1],
             latest_sbus_data.mappedChannels[1],
             STICK_DEADZONE);
    ROS_INFO("--------------------------------------------");
}

// SBUS callback
void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    printRemoteInfo();
    printCurrentState();

    // 保存前一次手动控制状态
    prev_remote_override = remote_override;

    // 1. Manual control judgment (only depends on CH1/CH2)
    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    if (ch1_active || ch2_active) {
        remote_override = true;
        
        // 将mappedChannels从[-100,100]映射到[-1,1]
        float angular_input = (ch1_active) ? (msg->mappedChannels[0] / 100.0f) : 0.0f;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] / 100.0f) : 0.0f;
        remote_cmd.linear.x = linear_x * linear_input;

        if (!prev_remote_override) {
            ROS_WARN("[Manual Control] Activated: CH1 or CH2 exceeds deadzone");
            // 重置积分项和误差
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
        }
    } else {
        remote_override = false;
        if (prev_remote_override) {
            ROS_INFO("[Manual Control] Deactivated: Sticks centered (within deadzone)");
            // 确保停止命令被发布
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0f;
            stop_cmd.angular.z = 0.0f;
            cmd_vel_pub.publish(stop_cmd);
        }
    }

    // 2. Return logic (triggered by CH5 up position)
    if (!remote_override) {
        uint16_t ch5_raw = msg->rawChannels[4];
        if (current_state == ControlState::WAIT_RETURN && ch5_raw > 1500) {
            current_state = ControlState::TURNING;
            // 重置积分项
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
            ROS_INFO("✅ CH5 up triggers return! Raw value: %d", ch5_raw);
        } else if ((current_state == ControlState::RETURN_HOME || 
                   current_state == ControlState::TURNING) && ch5_raw <= 100) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ CH5 down cancels return! Raw value: %d", ch5_raw);
        }
    }
}

// Odometry callback
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
    has_odom = true;
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_callback_time).toSec();
    last_callback_time = current_time;
    if (dt <= 0.0) dt = 0.01;

    // Read current position and heading
    float current_x = msg->pose.pose.position.x;
    float current_y = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, current_yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);

    // Initialization
    if (current_state == ControlState::INIT) {
        home_x = current_x;
        home_y = current_y;
        home_yaw = current_yaw;
        
        // 设置目标方向为当前方向（可以修改为任意期望的方向）
        target_direction = home_yaw;
        
        // 计算目标点
        target_x = home_x + target_distance * cos(target_direction);
        target_y = home_y + target_distance * sin(target_direction);
        
        current_state = ControlState::ALIGN_DIRECTION;
        ROS_INFO("Initialization complete! Home position: (%.2f, %.2f), Target direction: %.2f rad",
                 home_x, home_y, target_direction);
        return;
    }

    // Manual control priority
    if (remote_override) {
        // 手动控制时发布遥控命令
        cmd_vel_pub.publish(remote_cmd);
        ROS_INFO_THROTTLE(1, "[Manual Command] Linear: %.2f, Angular: %.2f",
                         remote_cmd.linear.x, remote_cmd.angular.z);
        return;
    }

    // Determine current target point
    float current_target_x, current_target_y;
    if (current_state == ControlState::AUTO_STRAIGHT || 
        current_state == ControlState::ALIGN_DIRECTION) {
        current_target_x = target_x;
        current_target_y = target_y;
    } else {
        current_target_x = home_x;
        current_target_y = home_y;
    }

    // Calculate remaining distance
    float remaining_dist = calculateDistance(current_x, current_y, current_target_x, current_target_y);

    // Target arrival handling
    if (remaining_dist < target_tolerance) {
        if (current_state == ControlState::AUTO_STRAIGHT || 
            current_state == ControlState::ALIGN_DIRECTION) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Reached target! Distance: %.2f meters, waiting for CH5 up to trigger return", remaining_dist);
        } else if (current_state == ControlState::RETURN_HOME) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Returned to home! Distance: %.2f meters", remaining_dist);
        }
        // 重置积分项
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0f;
        stop_cmd.angular.z = 0.0f;
        cmd_vel_pub.publish(stop_cmd);
        return;
    }

    // Handle direction alignment state
    if (current_state == ControlState::ALIGN_DIRECTION) {
        // 计算当前方向与目标方向的误差
        float angle_error = shortestAngularDistance(current_yaw, target_direction);
        
        // 如果方向正确，切换到直线行驶
        if (fabs(angle_error) < alignment_tolerance) {
            current_state = ControlState::AUTO_STRAIGHT;
            ROS_INFO("✅ Direction aligned! Starting straight movement");
            return;
        }
        
        // 原地转向对准方向
        geometry_msgs::Twist align_cmd;
        align_cmd.linear.x = 0.0f;
        align_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(align_cmd);
        
        ROS_INFO_THROTTLE(0.5, "[Aligning] Angle error: %.2f rad, Angular: %.2f", 
                         angle_error, align_cmd.angular.z);
        return;
    }

    // Handle turning state - face home before moving
    if (current_state == ControlState::TURNING) {
        // 重置积分项
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        
        // Calculate target direction (toward home)
        float home_direction = atan2(home_y - current_y, home_x - current_x);
        float angle_error = shortestAngularDistance(current_yaw, home_direction);
        
        // If facing the right direction, switch to RETURN_HOME
        if (fabs(angle_error) < angle_tolerance) {
            current_state = ControlState::RETURN_HOME;
            ROS_INFO("✅ Facing home direction, starting to move");
            return;
        }
        
        // Turn in place to face home
        geometry_msgs::Twist turn_cmd;
        turn_cmd.linear.x = 0.0f;
        turn_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(turn_cmd);
        
        ROS_INFO_THROTTLE(0.5, "[Turning] Angle error: %.2f rad, Angular: %.2f", 
                         angle_error, turn_cmd.angular.z);
        return;
    }

    // For AUTO_STRAIGHT and RETURN_HOME states, use PID control
    if (current_state == ControlState::AUTO_STRAIGHT || current_state == ControlState::RETURN_HOME) {
        // Optimized heading control near target
        float current_kp = kp;
        if (remaining_dist < near_target_threshold) {
            current_kp *= remaining_dist / near_target_threshold;
        }

        // Angle PID control (using dynamic kp)
        float target_yaw = atan2(current_target_y - current_y, current_target_x - current_x);
        float yaw_error = shortestAngularDistance(current_yaw, target_yaw);
        float yaw_error_rate = (yaw_error - last_yaw_error) / dt;
        last_yaw_error = yaw_error;

        // 积分项处理 - 只在误差方向一致时累积
        if (fabs(yaw_error) < 0.05f) {
            // 误差很小时，保持积分项不变
        } else if (yaw_error * last_yaw_error < 0) {
            integral_error = 0.0f;  // 重置积分项当误差方向改变
        } else {
            integral_error += yaw_error * dt;
            // 限制积分项防止积分饱和
            integral_error = std::max(std::min(integral_error, 1.0f), -1.0f);
        }

        // PID + 前馈补偿
        float angular_z = current_kp * yaw_error + ki * integral_error + kd * yaw_error_rate + feedforward_compensation;
        angular_z = std::max(std::min(angular_z, MAX_ANGULAR), -MAX_ANGULAR);  // Limit output

        // Speed control: smoother deceleration near target
        float linear_speed = linear_x;
        if (remaining_dist < near_target_threshold) {
            linear_speed *= pow(remaining_dist / near_target_threshold, 2);  // Quadratic deceleration
        }

        // Publish command
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed;
        cmd.angular.z = angular_z;
        cmd_vel_pub.publish(cmd);

        // State logging
        if (current_state == ControlState::AUTO_STRAIGHT) {
            ROS_INFO_THROTTLE(1, "[Auto Driving] Remaining: %.2fm | Yaw error: %.2f | Angular: %.2f | Integral: %.2f | Current kp: %.2f",
                             remaining_dist, yaw_error, angular_z, integral_error, current_kp);
        } else if (current_state == ControlState::RETURN_HOME) {
            ROS_INFO_THROTTLE(1, "[Returning Home] Remaining: %.2fm | Yaw error: %.2f | Angular: %.2f | Integral: %.2f | Current kp: %.2f",
                             remaining_dist, yaw_error, angular_z, integral_error, current_kp);
        }
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "straight_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Parameters adjustable via parameter server
    pnh.param<float>("linear_x", linear_x, 0.28f);
    pnh.param<float>("turn_speed", turn_speed, 0.5f);
    pnh.param<float>("target_distance", target_distance, 3.0f);
    pnh.param<float>("kp", kp, 1.5f);
    pnh.param<float>("ki", ki, 0.08f);
    pnh.param<float>("kd", kd, 0.3f);
    pnh.param<float>("feedforward_compensation", feedforward_compensation, 0.04f);
    pnh.param<float>("target_tolerance", target_tolerance, 0.15f);
    pnh.param<float>("near_target_threshold", near_target_threshold, 0.5f);
    pnh.param<float>("angle_tolerance", angle_tolerance, 0.1f);
    pnh.param<float>("alignment_tolerance", alignment_tolerance, 0.05f);

    last_callback_time = ros::Time::now();

    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ROS_INFO("Straight line controller started successfully!");
    ROS_INFO("Optimized parameters for tracked vehicle: kp=%.2f, ki=%.2f, kd=%.2f, feedforward=%.2f", 
             kp, ki, kd, feedforward_compensation);
    ROS_INFO("CH5 triggers return, CH1/CH2 for manual control (must exceed deadzone)");

    ros::spin();
    return 0;
}

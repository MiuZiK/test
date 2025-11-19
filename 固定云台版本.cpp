
/*基于固定云台版本编写  测试时，将遥控器 CH3 摇杆向上推（原始值 > 1500）LED 点亮，向下推（原始值 < 500）LED 熄灭，中间位置保持当前状态，不影响原有小车控制逻辑
    新增#include <wiringPi.h>头文件引用
    添加 LED 控制相关全局变量（引脚定义、状态变量、阈值）
    在main函数中初始化 wiringPi 并配置 LED 引脚为输出
    在printCurrentState中添加 LED 状态显示
    在printRemoteInfo中添加 CH3 通道信息打印
    在sbusCallback中实现 CH3 通道解析和 LED 控制逻辑
使用：
cmakelist文件加入
find_library(WIRINGPI_LIB wiringPi)
target_link_libraries(straight_controller_cpp ${WIRINGPI_LIB})

*/
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

// 全局变量
ros::Publisher cmd_vel_pub;
ros::Publisher gimbal_cmd_pub;
nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::INIT;
ros::Time last_callback_time;

// Control parameters
float linear_x = 0.28f;
float turn_speed = 0.5f;
float kp = 1.5f;
float ki = 0.08f;
float kd = 0.3f;
float feedforward_compensation = 0.04f;
float last_yaw_error = 0.0f;
float integral_error = 0.0f;
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

    ROS_INFO("[System State] Car mode: %s | Manual: %s", 
             state_str.c_str(), remote_override ? "Yes" : "No");
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

    ROS_INFO("----------------[Remote Control Info]----------------");
    ROS_INFO("CH5(SW A - Return): Raw=%d | State=%s", ch5_raw, ch5_state.c_str());
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

// 定时器回调：定期发送云台固定指令（垂直朝上）
void gimbalFixTimerCallback(const ros::TimerEvent& event) {
    std_msgs::String cmd_msg;
    cmd_msg.data = "0,90";  // 水平不动(0)，垂直朝上(100)
    gimbal_cmd_pub.publish(cmd_msg);
    ROS_DEBUG_THROTTLE(1.0, "[Gimbal] Fixed position command sent: %s", cmd_msg.data.c_str());
}

// SBUS callback
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
    ros::init(argc, argv, "integrated_car_gimbal_controller");//可以修改为自己命名的节点 编译注意下
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

    last_callback_time = ros::Time::now();

    // 订阅与发布器初始化
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    gimbal_cmd_pub = nh.advertise<std_msgs::String>("/ui_xy", 10);

    // 创建定时器，定期发送云台固定指令（垂直朝上）
    ros::Timer gimbal_fix_timer = nh.createTimer(ros::Duration(0.1), gimbalFixTimerCallback);

    // 节点启动时立即发送云台固定指令
    std_msgs::String init_cmd;
    init_cmd.data = "0,100";  // 水平不动(0)，垂直朝上(100)
    gimbal_cmd_pub.publish(init_cmd);
    ROS_INFO("[Gimbal] Initial fixed position command sent: %s", init_cmd.data.c_str());

    ROS_INFO("Integrated car & gimbal controller started successfully!");
    ROS_INFO("Gimbal is fixed to vertical up position.");

    ros::spin();
    return 0;
}

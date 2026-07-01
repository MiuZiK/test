
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
#include <wiringPi.h>

enum class ControlState {
    STANDBY,            // 待机状态：上电后默认，小车静止，等待遥控器启动指令
    ALIGN_DIRECTION,    // 自动模式：对准目标方向
    AUTO_STRAIGHT,      // 自动模式：直线行驶
    WAIT_RETURN,        // 自动模式：到达目标，等待返航指令
    RETURN_HOME,        // 自动模式：返航中
    TURNING             // 自动模式：调头朝向起点
};

enum class OperationMode {
    STANDBY_MODE,       // 待机模式（上电默认）
    AUTO_MODE,          // 自动巡检模式
    MANUAL_MODE         // 手动遥控模式
};

ros::Publisher cmd_vel_pub;
ros::Publisher gimbal_cmd_pub;
ros::Publisher mode_pub;
nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::STANDBY;
OperationMode op_mode = OperationMode::STANDBY_MODE;
ros::Time last_callback_time;

float linear_x = 0.33f;
float turn_speed = 0.5f;
float kp = 1.5f;
float ki = 0.08f;
float kd = 0.3f;
float feedforward_compensation = 0.04f;
float last_yaw_error = 0.0f;
float integral_error = 0.0f;
float target_distance = 16.0f;
float target_tolerance = 0.15f;
float near_target_threshold = 0.3f;
float angle_tolerance = 0.1f;
float alignment_tolerance = 0.05f;

float home_x = 0.0f, home_y = 0.0f;
float home_yaw = 0.0f;
float target_x = 0.0f, target_y = 0.0f;
float target_direction = 0.0f;

bool remote_override = false;
bool prev_remote_override = false;
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 15.0f;
const float MAX_ANGULAR = 0.5f;

const int LED_PIN = 10;
bool led_state = false;
const int CH10_LOW = 282;
const int CH10_HIGH = 1722;
const int CH10_TOLERANCE = 5;
int prev_ch10_state = -1;
int ch10_stable_count = 0;
const int REQUIRED_STABLE_COUNT = 3;

// CH6 (SW B) 遥控通道配置 - 用于启动/急停/模式切换
const int CH6_START_THRESHOLD = 1500;   // SW B UP: 启动自动模式
const int CH6_STOP_THRESHOLD = 1000;    // SW B DOWN: 急停/返回待机
int prev_ch6_state = 0;                 // 0=middle, 1=up, -1=down

float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

float shortestAngularDistance(float from, float to) {
    float diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

void publishZeroVelocity() {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0f;
    stop_cmd.linear.y = 0.0f;
    stop_cmd.linear.z = 0.0f;
    stop_cmd.angular.x = 0.0f;
    stop_cmd.angular.y = 0.0f;
    stop_cmd.angular.z = 0.0f;
    cmd_vel_pub.publish(stop_cmd);
}

void enterStandbyMode() {
    if (op_mode == OperationMode::STANDBY_MODE && current_state == ControlState::STANDBY) return;

    op_mode = OperationMode::STANDBY_MODE;
    current_state = ControlState::STANDBY;
    integral_error = 0.0f;
    last_yaw_error = 0.0f;
    publishZeroVelocity();
    ROS_WARN("[MODE] >>> STANDBY <<< - Car stopped, waiting for remote START command (CH6 UP)");
}

void enterAutoMode() {
    if (op_mode == OperationMode::AUTO_MODE) return;

    if (!has_odom) {
        ROS_WARN("[MODE] Cannot enter AUTO mode: no odometry data yet");
        return;
    }

    op_mode = OperationMode::AUTO_MODE;
    current_state = ControlState::INIT;
    integral_error = 0.0f;
    last_yaw_error = 0.0f;
    ROS_INFO("[MODE] >>> AUTO <<< - Auto inspection started, gimbal locked up");
}

void enterManualMode() {
    if (op_mode == OperationMode::MANUAL_MODE) return;

    op_mode = OperationMode::MANUAL_MODE;
    ROS_INFO("[MODE] >>> MANUAL <<< - Remote control active, auto logic paused");
}

void printSystemStatus() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 2.0) return;
    last_print = ros::Time::now();

    std::string state_str, mode_str;
    switch (current_state) {
        case ControlState::STANDBY:     state_str = "STANDBY"; break;
        case ControlState::ALIGN_DIRECTION: state_str = "ALIGN"; break;
        case ControlState::AUTO_STRAIGHT:   state_str = "DRIVE"; break;
        case ControlState::WAIT_RETURN:    state_str = "WAIT_RET"; break;
        case ControlState::RETURN_HOME:    state_str = "RETURN"; break;
        case ControlState::TURNING:        state_str = "TURNING"; break;
    }
    switch (op_mode) {
        case OperationMode::STANDBY_MODE: mode_str = "STANDBY"; break;
        case OperationMode::AUTO_MODE:    mode_str = "AUTO"; break;
        case OperationMode::MANUAL_MODE:  mode_str = "MANUAL"; break;
    }

    ROS_INFO("[STATUS] Mode=%s | State=%s | LED=%s | Odom=%s",
             mode_str.c_str(), state_str.c_str(),
             led_state ? "ON" : "OFF", has_odom ? "OK" : "WAIT");
}

void gimbalFixTimerCallback(const ros::TimerEvent& event) {
    std_msgs::String cmd_msg;
    cmd_msg.data = "0,90";
    gimbal_cmd_pub.publish(cmd_msg);
}

void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    prev_remote_override = remote_override;

    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    // ========== CH6 (SW B): 启动/急停/模式切换 ==========
    uint16_t ch6_raw = msg->rawChannels[5];
    int ch6_state = 0;
    if (ch6_raw > CH6_START_THRESHOLD) ch6_state = 1;       // UP
    else if (ch6_raw < CH6_STOP_THRESHOLD) ch6_state = -1;   // DOWN

    if (ch6_state != prev_ch6_state) {
        if (ch6_state == 1) {
            // SW B UP: 根据当前模式决定行为
            if (op_mode == OperationMode::STANDBY_MODE) {
                enterAutoMode();
            } else if (op_mode == OperationMode::AUTO_MODE || op_mode == OperationMode::MANUAL_MODE) {
                // 运行中再次拨UP = 急停，回到待机
                ROS_WARN("[CH6] EMERGENCY STOP triggered by SW B UP while running!");
                enterStandbyMode();
            }
        } else if (ch6_state == -1) {
            // SW B DOWN: 任何状态下都急停回待机
            ROS_WARN("[CH6] EMERGENCY STOP triggered by SW B DOWN!");
            enterStandbyMode();
        }
        prev_ch6_state = ch6_state;
    }

    // ========== 手动遥控检测 (CH1/CH2 摇杆) ==========
    if (ch1_active || ch2_active) {
        if (op_mode != OperationMode::MANUAL_MODE) {
            enterManualMode();
        }
        remote_override = true;

        float angular_input = (ch1_active) ? (msg->mappedChannels[0] / 100.0f) : 0.0f;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] / 100.0f) : 0.0f;
        remote_cmd.linear.x = linear_x * linear_input;

        if (!prev_remote_override) {
            ROS_INFO("[Manual] Sticks activated - MANUAL mode engaged");
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
        }
    } else {
        remote_override = false;
        if (prev_remote_override && op_mode == OperationMode::MANUAL_MODE) {
            ROS_INFO("[Manual] Sticks centered - returning to previous mode");
            if (has_odom) {
                enterAutoMode();
            } else {
                enterStandbyMode();
            }
            publishZeroVelocity();
        }
    }

    // ========== CH10: LED 控制 ==========
    uint16_t ch10_raw = msg->rawChannels[9];
    int current_ch10_state = -1;
    if (abs(ch10_raw - CH10_HIGH) <= CH10_TOLERANCE) current_ch10_state = 1;
    else if (abs(ch10_raw - CH10_LOW) <= CH10_TOLERANCE) current_ch10_state = 0;

    if (current_ch10_state != -1) {
        if (current_ch10_state == prev_ch10_state) {
            ch10_stable_count = 0;
        } else {
            ch10_stable_count++;
            if (ch10_stable_count >= REQUIRED_STABLE_COUNT) {
                led_state = (current_ch10_state == 1);
                digitalWrite(LED_PIN, led_state ? HIGH : LOW);
                ROS_INFO("[LED] %s (CH10=%d)", led_state ? "ON" : "OFF", ch10_raw);
                prev_ch10_state = current_ch10_state;
                ch10_stable_count = 0;
            }
        }
    }

    // ========== CH5: 返航触发 (仅在自动模式下有效) ==========
    if (op_mode == OperationMode::AUTO_MODE && !remote_override) {
        uint16_t ch5_raw = msg->rawChannels[4];
        if (current_state == ControlState::WAIT_RETURN && ch5_raw > 1500) {
            current_state = ControlState::TURNING;
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
            ROS_INFO("[CH5] Return triggered! Starting turn to home");
        } else if ((current_state == ControlState::RETURN_HOME ||
                   current_state == ControlState::TURNING) && ch5_raw <= 100) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("[CH5] Return cancelled, back to WAIT_RETURN");
            publishZeroVelocity();
        }
    }

    printSystemStatus();
}

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

    // 待机模式：不执行任何自动逻辑
    if (op_mode == OperationMode::STANDBY_MODE) {
        publishZeroVelocity();
        return;
    }

    // 手动模式：只发布遥控指令
    if (op_mode == OperationMode::MANUAL_MODE) {
        if (remote_override) {
            cmd_vel_pub.publish(remote_cmd);
        } else {
            publishZeroVelocity();
        }
        return;
    }

    // ======== 以下为自动模式逻辑 ========
    if (current_state == ControlState::INIT) {
        home_x = current_x;
        home_y = current_y;
        home_yaw = current_yaw;
        target_direction = home_yaw;
        target_x = home_x + target_distance * cos(target_direction);
        target_y = home_y + target_distance * sin(target_direction);
        current_state = ControlState::ALIGN_DIRECTION;
        ROS_INFO("[AUTO] Home recorded: (%.2f, %.2f), Target: (%.2f, %.2f)",
                 home_x, home_y, target_x, target_y);
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
            ROS_INFO("[AUTO] Target reached (%.2fm)! Waiting for CH5 to return or CH6 to stop",
                     target_distance);
        } else if (current_state == ControlState::RETURN_HOME) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("[AUTO] Returned home safely!");
        }
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        publishZeroVelocity();
        return;
    }

    if (current_state == ControlState::ALIGN_DIRECTION) {
        float angle_error = shortestAngularDistance(current_yaw, target_direction);
        if (fabs(angle_error) < alignment_tolerance) {
            current_state = ControlState::AUTO_STRAIGHT;
            ROS_INFO("[AUTO] Aligned! Driving straight...");
            return;
        }
        geometry_msgs::Twist align_cmd;
        align_cmd.linear.x = 0.0f;
        align_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(align_cmd);
        return;
    }

    if (current_state == ControlState::TURNING) {
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        float home_direction = atan2(home_y - current_y, home_x - current_x);
        float angle_error = shortestAngularDistance(current_yaw, home_direction);
        if (fabs(angle_error) < angle_tolerance) {
            current_state = ControlState::RETURN_HOME;
            ROS_INFO("[AUTO] Facing home, driving back...");
            return;
        }
        geometry_msgs::Twist turn_cmd;
        turn_cmd.linear.x = 0.0f;
        turn_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(turn_cmd);
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

        ROS_INFO_THROTTLE(1.0, "[AUTO] %s | Remaining: %.2fm | YawErr: %.2f",
                         (current_state == ControlState::AUTO_STRAIGHT) ? "DRIVE" : "RETURN",
                         remaining_dist, yaw_error);
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "undercarriage_inspection_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    if (wiringPiSetup() == -1) {
        ROS_ERROR("Failed to initialize wiringPi!");
        return -1;
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pnh.param<float>("linear_x", linear_x, 0.33f);
    pnh.param<float>("turn_speed", turn_speed, 0.5f);
    pnh.param<float>("target_distance", target_distance, 16.0f);
    pnh.param<float>("kp", kp, 1.5f);
    pnh.param<float>("ki", ki, 0.08f);
    pnh.param<float>("kd", kd, 0.3f);
    pnh.param<float>("feedforward_compensation", feedforward_compensation, 0.04f);

    last_callback_time = ros::Time::now();

    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    gimbal_cmd_pub = nh.advertise<std_msgs::String>("/ui_xy", 10);

    ros::Timer gimbal_fix_timer = nh.createTimer(ros::Duration(0.1), gimbalFixTimerCallback);

    std_msgs::String init_cmd;
    init_cmd.data = "0,90";
    gimbal_cmd_pub.publish(init_cmd);

    ROS_INFO("========================================");
    ROS_INFO(" Undercarriage Inspection Controller v2.0 ");
    ROS_INFO("========================================");
    ROS_INFO(" Default mode: STANDBY (car does NOT move)");
    ROS_INFO(" CH6 (SW B) UP   -> START auto inspection");
    ROS_INFO(" CH6 (SW B) DOWN -> EMERGENCY STOP (anytime)");
    ROS_INFO(" CH6 (SW B) UP   -> EMERGENCY STOP (while running)");
    ROS_INFO(" CH1/CH2 sticks  -> MANUAL mode (auto paused)");
    ROS_INFO(" CH5 (SW A)      -> Trigger return (in auto mode)");
    ROS_INFO(" CH10            -> LED toggle");
    ROS_INFO(" Gimbal: locked vertical up (90 deg)");
    ROS_INFO("========================================");

    ros::spin();
    return 0;
}

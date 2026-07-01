//910test good
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

/*911 good test*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>

// System state enumeration
enum class ControlState {
    INIT,           // Initialization
    AUTO_STRAIGHT,  // Auto straight driving
    WAIT_RETURN,    // Waiting for return trigger
    RETURN_HOME,    // Returning to home position
    TURNING         // Turning to face home
};

// Global variables
ros::Publisher cmd_vel_pub;
nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::INIT;

// Control parameters (optimized for tracked vehicle)
float linear_x = 0.28f;            // Driving speed
float turn_speed = 0.4f;           // Turning speed
float kp = 1.5f;                   // Increased proportional gain for faster response
float ki = 0.08f;                  // Increased integral gain to eliminate steady-state error
float kd = 0.3f;                   // Increased derivative gain for dampening
float feedforward_compensation = 0.05f;  // 增加前馈补偿
float last_yaw_error = 0.0f;
float integral_error = 0.0f;       // 积分误差累积
ros::Time last_callback_time;
float target_distance = 12.0f;
float target_tolerance = 0.15f;
float near_target_threshold = 0.3f;  // Deceleration threshold near target 
float angle_tolerance = 0.1f;        // Angle tolerance for turning (rad)

// Coordinate parameters
float home_x = 0.0f, home_y = 0.0f;
float home_yaw = 0.0f;
float target_x = 0.0f, target_y = 0.0f;

// Remote control related (added deadzone parameter)
bool remote_override = false;
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 0.15f;  // Joystick deadzone (increased from 0.1 to avoid minor deviations)
const float MAX_ANGULAR = 0.5f;      // Maximum angular velocity (reduced from 0.6 to prevent fast rotation)

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
        case ControlState::AUTO_STRAIGHT: state_str = "AUTO_STRAIGHT"; break;
        case ControlState::WAIT_RETURN: state_str = "WAIT_RETURN"; break;
        case ControlState::RETURN_HOME: state_str = "RETURN_HOME"; break;
        case ControlState::TURNING: state_str = "TURNING"; break;
    }
    ROS_INFO("[System State] Current mode: %s | Manual control: %s", 
             state_str.c_str(), remote_override ? "Yes" : "No");
}

// Print remote control information (fixed format specifiers)
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
    // Fixed: Use %d instead of %.2f for mappedChannels (integer values)
    ROS_INFO("CH1(Steering): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[0],
             latest_sbus_data.mappedChannels[0],  // This is an integer
             STICK_DEADZONE);
    ROS_INFO("CH2(Throttle): Raw=%d | Mapped=%d (Deadzone=±%.2f)",
             latest_sbus_data.rawChannels[1],
             latest_sbus_data.mappedChannels[1],  // This is an integer
             STICK_DEADZONE);
    ROS_INFO("--------------------------------------------");
}

// SBUS callback (optimized manual control logic)
void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    printRemoteInfo();
    printCurrentState();

    // 1. Manual control judgment (only depends on CH1/CH2)
    bool prev_override = remote_override;
    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    if (ch1_active || ch2_active) {
        remote_override = true;
        // Steering command: deadzone handling + limiting
        float angular_input = (ch1_active) ? (msg->mappedChannels[0] * 2 - 1.0) : 0.0;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        
        // Throttle command: deadzone handling
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] * 2 - 1.0) : 0.0;
        remote_cmd.linear.x = linear_x * linear_input;

        if (!prev_override) {
            ROS_WARN("[Manual Control] Activated: CH1 or CH2 exceeds deadzone");
        }
    } else {
        remote_override = false;
        if (prev_override) {
            ROS_INFO("[Manual Control] Deactivated: Sticks centered (within deadzone)");
        }
    }

    // 2. Return logic (triggered by CH5 up position)
    if (!remote_override) {
        uint16_t ch5_raw = msg->rawChannels[4];
        if (current_state == ControlState::WAIT_RETURN && ch5_raw > 1500) {
            current_state = ControlState::TURNING;
            // 重置积分项
            integral_error = 0.0f;
            ROS_INFO("✅ CH5 up triggers return! Raw value: %d", ch5_raw);
        } else if ((current_state == ControlState::RETURN_HOME || 
                   current_state == ControlState::TURNING) && ch5_raw <= 100) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ CH5 down cancels return! Raw value: %d", ch5_raw);
        }
    }
}

// Odometry callback (optimized heading control near target)
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
    has_odom = true;
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_callback_time).toSec();
    last_callback_time = current_time;
    if (dt <= 0.0) dt = 0.01;

    // Initialization
    if (current_state == ControlState::INIT) {
        home_x = msg->pose.pose.position.x;
        home_y = msg->pose.pose.position.y;
        
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        home_yaw = yaw;

        transformLocalToGlobal(target_distance, 0.0f, target_x, target_y);
        current_state = ControlState::AUTO_STRAIGHT;
        // 重置积分项
        integral_error = 0.0f;
        ROS_INFO("Initialization complete! Home position: (%.2f, %.2f), Target distance: %.2f meters",
                 home_x, home_y, target_distance);
        return;
    }

    // Manual control priority
    if (remote_override) {
        // 手动控制时重置积分项
        integral_error = 0.0f;
        cmd_vel_pub.publish(remote_cmd);
        ROS_INFO_THROTTLE(1, "[Manual Command] Linear: %.2f, Angular: %.2f",
                         remote_cmd.linear.x, remote_cmd.angular.z);
        return;
    }

    // Read current position and heading
    float current_x = msg->pose.pose.position.x;
    float current_y = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, current_yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);

    // Determine current target point
    float current_target_x, current_target_y;
    if (current_state == ControlState::AUTO_STRAIGHT) {
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
        if (current_state == ControlState::AUTO_STRAIGHT) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Reached target! Distance: %.2f meters, waiting for CH5 up to trigger return", remaining_dist);
        } else if (current_state == ControlState::RETURN_HOME) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ Returned to home! Distance: %.2f meters", remaining_dist);
        }
        // 重置积分项
        integral_error = 0.0f;
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0f;
        stop_cmd.angular.z = 0.0f;
        cmd_vel_pub.publish(stop_cmd);
        return;
    }

    // Handle turning state - face home before moving
    if (current_state == ControlState::TURNING) {
        // 重置积分项
        integral_error = 0.0f;
        
        // Calculate target direction (toward home)
        float target_direction = atan2(home_y - current_y, home_x - current_x);
        float angle_error = shortestAngularDistance(current_yaw, target_direction);
        
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
    pnh.param<float>("turn_speed", turn_speed, 0.4f);
    pnh.param<float>("target_distance", target_distance, 12.0f);
    pnh.param<float>("kp", kp, 1.5f);  // Increased for faster response
    pnh.param<float>("ki", ki, 0.08f);  // Increased to eliminate steady-state error
    pnh.param<float>("kd", kd, 0.3f);   // Increased for better dampening
    pnh.param<float>("feedforward_compensation", feedforward_compensation, 0.05f);  // 增加前馈补偿
    pnh.param<float>("target_tolerance", target_tolerance, 0.15f);
    pnh.param<float>("near_target_threshold", near_target_threshold, 0.5f);
    pnh.param<float>("angle_tolerance", angle_tolerance, 0.1f);

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



/*9.10DS ver3.0*/
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
float turn_speed = 0.3f;           // Turning speed
float kp = 1.5f;                   // Proportional gain
float ki = 0.08f;                  // Integral gain
float kd = 0.3f;                   // Derivative gain
float feedforward_compensation = 0.03f;  // 前馈补偿
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
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 0.15f;  // Joystick deadzone
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

    // 1. Manual control judgment (only depends on CH1/CH2)
    bool prev_override = remote_override;
    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    if (ch1_active || ch2_active) {
        remote_override = true;
        // Steering command: deadzone handling + limiting
        float angular_input = (ch1_active) ? (msg->mappedChannels[0] * 2 - 1.0) : 0.0;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        
        // Throttle command: deadzone handling
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] * 2 - 1.0) : 0.0;
        remote_cmd.linear.x = linear_x * linear_input;

        if (!prev_override) {
            ROS_WARN("[Manual Control] Activated: CH1 or CH2 exceeds deadzone");
        }
    } else {
        remote_override = false;
        if (prev_override) {
            ROS_INFO("[Manual Control] Deactivated: Sticks centered (within deadzone)");
        }
    }

    // 2. Return logic (triggered by CH5 up position)
    if (!remote_override) {
        uint16_t ch5_raw = msg->rawChannels[4];
        if (current_state == ControlState::WAIT_RETURN && ch5_raw > 1500) {
            current_state = ControlState::TURNING;
            // 重置积分项
            integral_error = 0.0f;
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
        // 手动控制时重置积分项
        integral_error = 0.0f;
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
    pnh.param<float>("turn_speed", turn_speed, 0.3f);
    pnh.param<float>("target_distance", target_distance, 3.0f);
    pnh.param<float>("kp", kp, 1.5f);
    pnh.param<float>("ki", ki, 0.08f);
    pnh.param<float>("kd", kd, 0.3f);
    pnh.param<float>("feedforward_compensation", feedforward_compensation, 0.03f);
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

//699 9.13?
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <iomanip>

// 云台驱动相关定义
typedef struct {
    uint8_t sysn[2];
    struct {
        uint8_t trig;
        uint8_t valu;
    } cmd;
    struct {
        int8_t fl_sens;
    } aux;
    struct {
        uint8_t go_zero;
        uint8_t wk_mode;
        uint8_t op_type;
        int16_t op_valu;
    } gbc[2];
    uint8_t crc[2];
} Gcu2GbcPkt_t;

typedef struct {
    uint8_t sync[2];
    uint8_t fw_ver;
    uint8_t hw_err;
    uint8_t inv_flag;
    uint8_t gbc_stat;
    uint8_t tca_flag;
    struct {
        uint8_t stat;
        uint8_t valu;
    } cmd;
    int16_t cam_rate[3];
    int32_t cam_angl[3];
    int32_t mtr_angl[3];
    uint8_t crc[2];
} Gbc2GcuPkt_t;

// 函数声明 - 解决未声明问题
uint16_t CalculateCrc16(const uint8_t* data, uint32_t len);
void yuntai_start();
void yuntai_stop();
void yuntai_ctrl(int roll, int pitch);
void read_yuntai_feedback();
void setGimbalUp();  // 关键修复：提前声明setGimbalUp函数

// 系统状态枚举
enum class ControlState {
    INIT,               // 初始化
    ALIGN_DIRECTION,    // 对准目标方向
    AUTO_STRAIGHT,      // 自动直线行驶
    WAIT_RETURN,        // 等待返航触发
    RETURN_HOME,        // 返航中
    TURNING             // 转向回家方向
};

// 云台控制参数配置
struct GimbalConfig {
    int roll_channel = 3;     // CH4控制水平旋转
    int pitch_channel = 2;    // CH3控制垂直旋转
    int mode_channel = 5;     // CH6控制启动/停止
    float deadzone = 15.0f;   // 摇杆死区
    int min_pitch = -90;      // 垂直最小角度
    int max_pitch = 90;       // 垂直最大角度（朝上为90）
    int min_roll = -90;       // 水平最小角度
    int max_roll = 90;        // 水平最大角度
    int mode_low = 1000;      // 模式通道下档阈值
    int mode_high = 2000;     // 模式通道上档阈值
    int up_pitch = 90;        // 垂直向上角度（核心参数）
    int center_roll = 0;      // 水平中心角度
    float control_sensitivity = 0.5f;  // 遥控灵敏度
} gimbal_cfg;

// 云台状态枚举
enum GimbalMode {
    STOPPED,   // 停止
    STARTED,   // 启动中
    RUNNING    // 运行中
};

// 全局变量
ros::Publisher cmd_vel_pub;
ros::Publisher yuntai_ret_info;      // 云台返回信息发布
ros::Timer return_timer;             // 自动回位定时器
serial::Serial sp;                   // 云台串口

nav_msgs::Odometry current_odom;
bool has_odom = false;
ControlState current_state = ControlState::INIT;
GimbalMode gimbal_mode = STOPPED;
int current_roll = 0;         // 当前水平角度
int current_pitch = 0;        // 当前垂直角度
bool remote_controlling = false;  // 标记是否正在遥控云台
ros::Time last_remote_input_time;  // 最后一次遥控输入时间
float AUTO_RETURN_DELAY = 0.5f;    // 自动回位延迟(秒)
ros::Time last_gimbal_cmd_time;
const float GIMBAL_CMD_INTERVAL = 0.1f;  // 指令发送间隔
int gimbal_send_count = 0;    // 启动指令发送计数
int cnt = 0;                  // 云台命令触发计数

// 小车控制参数
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

// 坐标参数
float home_x = 0.0f, home_y = 0.0f;
float home_yaw = 0.0f;
float target_x = 0.0f, target_y = 0.0f;
float target_direction = 0.0f;

// 遥控相关
bool remote_override = false;
bool prev_remote_override = false;
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 15.0f;
const float MAX_ANGULAR = 0.5f;

// ---------------------------- 云台驱动函数 ----------------------------
uint16_t CalculateCrc16(const uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 云台启动指令
void yuntai_start() {
    ROS_INFO_STREAM("发送云台启动指令");
    if (sp.isOpen()) {		
        if (cnt > 255) cnt = 0;
        Gcu2GbcPkt_t pkt = {0};
        pkt.sysn[0] = 0xA9;
        pkt.sysn[1] = 0x5B;
        pkt.cmd.trig = cnt++;  // 递增触发计数
        pkt.cmd.valu = 2;      // 启动命令码
        pkt.aux.fl_sens = 0;
        pkt.gbc[0].go_zero = 0;
        pkt.gbc[1].go_zero = 0; 
        uint16_t crc = CalculateCrc16((uint8_t*)&pkt, sizeof(Gcu2GbcPkt_t)-2);
        pkt.crc[0] = (crc >> 8) & 0xFF;
        pkt.crc[1] = crc & 0xFF;
        sp.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(Gcu2GbcPkt_t));
    } else {
        ROS_ERROR_STREAM("云台串口未打开，无法发送启动指令");
    }
}

// 云台停止指令
void yuntai_stop() {
    ROS_INFO_STREAM("发送云台停止指令");
    if (sp.isOpen()) {		
        Gcu2GbcPkt_t pkt = {0};
        pkt.sysn[0] = 0xA9;
        pkt.sysn[1] = 0x5B;
        pkt.cmd.trig = 0;
        pkt.cmd.valu = 3;      // 停止命令码
        pkt.aux.fl_sens = 0;
        pkt.gbc[0].go_zero = 0;
        pkt.gbc[1].go_zero = 0; 
        uint16_t crc = CalculateCrc16((uint8_t*)&pkt, sizeof(Gcu2GbcPkt_t)-2);
        pkt.crc[0] = (crc >> 8) & 0xFF;
        pkt.crc[1] = crc & 0xFF;
        sp.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(Gcu2GbcPkt_t));
    } else {
        ROS_ERROR_STREAM("云台串口未打开，无法发送停止指令");
    }
}

// 云台角度控制指令
void yuntai_ctrl(int roll, int pitch) {
    if (sp.isOpen()) {				
        if (cnt > 255) cnt = 0;
        Gcu2GbcPkt_t pkt = {0};
        pkt.sysn[0] = 0xA9;
        pkt.sysn[1] = 0x5B;
        pkt.cmd.trig = cnt++;  // 递增触发计数
        pkt.cmd.valu = 4;      // 手动控制命令码
        pkt.aux.fl_sens = 0;
        pkt.gbc[0].go_zero = 0;
        pkt.gbc[0].wk_mode = 0;
        pkt.gbc[0].op_type = 0;
        pkt.gbc[0].op_valu = roll * 100;  // 水平控制量（×100）
        pkt.gbc[1].go_zero = 0; 
        pkt.gbc[1].wk_mode = 0;
        pkt.gbc[1].op_type = 0;
        pkt.gbc[1].op_valu = pitch * 100; // 垂直控制量（×100）
        uint16_t crc = CalculateCrc16((uint8_t*)&pkt, sizeof(Gcu2GbcPkt_t)-2);
        pkt.crc[0] = (crc >> 8) & 0xFF;
        pkt.crc[1] = crc & 0xFF;
        sp.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(Gcu2GbcPkt_t));
        ROS_DEBUG("发送云台角度指令: 水平=%d, 垂直=%d", roll, pitch);
    } else {
        ROS_ERROR_STREAM("云台串口未打开，无法发送角度指令");
    }
}

// 读取云台返回信息
void read_yuntai_feedback() {
    if (sp.isOpen() && sp.available() > 0) {
        uint8_t buffer[1024];
        size_t n = sp.read(buffer, sizeof(buffer));
        if (n >= sizeof(Gbc2GcuPkt_t)) {
            Gbc2GcuPkt_t* ptr = (Gbc2GcuPkt_t*)buffer;
            std::stringstream ss;
            ss << "云台反馈: "
               << "sync=0x" << std::hex << std::setw(2) << (int)ptr->sync[0] 
               << ",0x" << std::setw(2) << (int)ptr->sync[1] << " | "
               << "状态=0x" << (int)ptr->gbc_stat << " | "
               << "当前角度(deg): " 
               << std::dec << ptr->cam_angl[0]/100.0 << "," 
               << ptr->cam_angl[1]/100.0 << "," 
               << ptr->cam_angl[2]/100.0;
            std_msgs::String msg;
            msg.data = ss.str();
            yuntai_ret_info.publish(msg);
        }
    }
}

// 强制云台朝上（核心功能）
void setGimbalUp() {
    if (gimbal_mode != RUNNING) {
        ROS_WARN("[Gimbal] 未启动，无法设置朝上角度");
        return;
    }
    
    // 计算目标角度（确保在硬件限制范围内）
    int target_pitch = std::max(std::min(gimbal_cfg.up_pitch, gimbal_cfg.max_pitch), gimbal_cfg.min_pitch);
    int target_roll = std::max(std::min(gimbal_cfg.center_roll, gimbal_cfg.max_roll), gimbal_cfg.min_roll);
    
    // 发送朝上指令
    if (target_roll != current_roll || target_pitch != current_pitch) {
        yuntai_ctrl(target_roll, target_pitch);
        current_roll = target_roll;
        current_pitch = target_pitch;
        last_gimbal_cmd_time = ros::Time::now();
        ROS_INFO("[Gimbal] 回到朝上角度: 水平=%d, 垂直=%d", target_roll, target_pitch);
    }
    
    remote_controlling = false;
}

// ---------------------------- 小车控制逻辑 ----------------------------
// 自动回位定时器回调
void returnTimerCallback(const ros::TimerEvent& event) {
    if (gimbal_mode == RUNNING && !remote_controlling) {
        if ((ros::Time::now() - last_remote_input_time).toSec() > AUTO_RETURN_DELAY) {
            setGimbalUp();  // 现在这个函数已经在前面声明过了
        }
    }
}

// 计算两点距离
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 计算最短角度差
float shortestAngularDistance(float from, float to) {
    float diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

// 将摇杆值映射为云台角度增量
int mapStickToAngle(int stick_value) {
    return static_cast<int>(stick_value * gimbal_cfg.control_sensitivity);
}

// 处理云台模式控制（启动/停止）
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
        if (new_mode == STARTED) {
            yuntai_start();
            gimbal_send_count = 3;  // 发送3次启动指令确保生效
            ROS_INFO("[Gimbal] 启动中（CH6上档）");
        } else if (new_mode == STOPPED) {
            yuntai_stop();
            ROS_INFO("[Gimbal] 已停止（CH6下档）");
        } else if (new_mode == RUNNING) {
            ROS_INFO("[Gimbal] 运行中（CH6中档）");
            setGimbalUp();  // 进入运行模式后立即朝上
        }
        gimbal_mode = new_mode;
    }
    
    // 启动指令重发机制
    if (gimbal_send_count > 0) {
        yuntai_start();
        gimbal_send_count--;
    }
}

// 处理云台旋转控制（遥控调整+自动回位）
void handleGimbalControl(const sbus_serial::Sbus::ConstPtr& msg) {
    if (gimbal_mode != RUNNING) return;

    // 读取遥控输入
    int raw_roll = msg->mappedChannels[gimbal_cfg.roll_channel];
    int raw_pitch = msg->mappedChannels[gimbal_cfg.pitch_channel];
    
    // 判断是否有有效遥控输入（超过死区）
    bool roll_active = fabs(raw_roll) > gimbal_cfg.deadzone;
    bool pitch_active = fabs(raw_pitch) > gimbal_cfg.deadzone;
    
    if (roll_active || pitch_active) {
        // 有有效输入：根据遥控调整角度
        remote_controlling = true;
        last_remote_input_time = ros::Time::now();
        
        // 计算目标角度
        int target_roll = current_roll;
        int target_pitch = current_pitch;
        
        if (roll_active) {
            target_roll = std::max(std::min(
                current_roll + mapStickToAngle(raw_roll), 
                gimbal_cfg.max_roll), gimbal_cfg.min_roll);
        }
        
        if (pitch_active) {
            target_pitch = std::max(std::min(
                current_pitch + mapStickToAngle(raw_pitch), 
                gimbal_cfg.max_pitch), gimbal_cfg.min_pitch);
        }
        
        // 发送角度指令
        if (target_roll != current_roll || target_pitch != current_pitch) {
            yuntai_ctrl(target_roll, target_pitch);
            current_roll = target_roll;
            current_pitch = target_pitch;
            last_gimbal_cmd_time = ros::Time::now();
            ROS_DEBUG("[Gimbal] 遥控调整: 水平=%d, 垂直=%d", target_roll, target_pitch);
        }
    } else {
        // 无有效输入：自动回位
        if (remote_controlling) {
            if ((ros::Time::now() - last_remote_input_time).toSec() > AUTO_RETURN_DELAY) {
                setGimbalUp();
            }
        } else {
            // 定期发送朝上指令，确保状态稳定
            if ((ros::Time::now() - last_gimbal_cmd_time).toSec() > 1.0) {
                setGimbalUp();
            }
        }
    }
}

// 打印系统状态
void printCurrentState() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 1.0) return;
    last_print = ros::Time::now();

    std::string state_str;
    switch (current_state) {
        case ControlState::INIT: state_str = "初始化"; break;
        case ControlState::ALIGN_DIRECTION: state_str = "对准方向"; break;
        case ControlState::AUTO_STRAIGHT: state_str = "自动直行"; break;
        case ControlState::WAIT_RETURN: state_str = "等待返航"; break;
        case ControlState::RETURN_HOME: state_str = "返航中"; break;
        case ControlState::TURNING: state_str = "转向中"; break;
    }

    std::string gimbal_mode_str;
    switch (gimbal_mode) {
        case STOPPED: gimbal_mode_str = "已停止"; break;
        case STARTED: gimbal_mode_str = "启动中"; break;
        case RUNNING: 
            gimbal_mode_str = remote_controlling ? "遥控调整中" : "朝上锁定";
            break;
    }

    ROS_INFO("[系统状态] 小车模式: %s | 手动控制: %s | 云台模式: %s (当前角度: %d,%d)", 
             state_str.c_str(), remote_override ? "是" : "否", 
             gimbal_mode_str.c_str(), current_roll, current_pitch);
}

// SBUS遥控回调（处理小车控制和云台指令）
void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    printCurrentState();
    read_yuntai_feedback();  // 读取云台反馈

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
            ROS_WARN("[手动控制] 激活: CH1或CH2超过死区");
            integral_error = 0.0f;
            last_yaw_error = 0.0f;
        }
    } else {
        remote_override = false;
        if (prev_remote_override) {
            ROS_INFO("[手动控制] 关闭: 摇杆回中");
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
            ROS_INFO("✅ CH5上触发返航!");
        } else if ((current_state == ControlState::RETURN_HOME || 
                   current_state == ControlState::TURNING) && ch5_raw <= 100) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ CH5下取消返航!");
        }
    }

    // 云台控制逻辑（模式切换和角度控制）
    handleGimbalMode(msg->rawChannels[gimbal_cfg.mode_channel]);
    handleGimbalControl(msg);
}

// 里程计回调（小车自动行驶控制）
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
        ROS_INFO("初始化完成! 起点位置: (%.2f, %.2f)", home_x, home_y);
        
        // 初始化后设置云台朝上
        if (gimbal_mode == RUNNING) {
            setGimbalUp();
        }
        return;
    }

    if (remote_override) {
        cmd_vel_pub.publish(remote_cmd);
        ROS_INFO_THROTTLE(1, "[手动指令] 线速度: %.2f, 角速度: %.2f",
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
            ROS_INFO("✅ 到达目标点! 等待CH5上触发返航");
        } else if (current_state == ControlState::RETURN_HOME) {
            current_state = ControlState::WAIT_RETURN;
            ROS_INFO("✅ 回到起点!");
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
            ROS_INFO("✅ 方向对准完成! 开始直线行驶");
            return;
        }
        geometry_msgs::Twist align_cmd;
        align_cmd.linear.x = 0.0f;
        align_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(align_cmd);
        ROS_INFO_THROTTLE(0.5, "[对准中] 角度误差: %.2f 弧度", angle_error);
        return;
    }

    if (current_state == ControlState::TURNING) {
        integral_error = 0.0f;
        last_yaw_error = 0.0f;
        float home_direction = atan2(home_y - current_y, home_x - current_x);
        float angle_error = shortestAngularDistance(current_yaw, home_direction);
        if (fabs(angle_error) < angle_tolerance) {
            current_state = ControlState::RETURN_HOME;
            ROS_INFO("✅ 已对准起点方向，开始返航");
            return;
        }
        geometry_msgs::Twist turn_cmd;
        turn_cmd.linear.x = 0.0f;
        turn_cmd.angular.z = (angle_error > 0) ? turn_speed : -turn_speed;
        cmd_vel_pub.publish(turn_cmd);
        ROS_INFO_THROTTLE(0.5, "[转向中] 角度误差: %.2f 弧度", angle_error);
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
            // 误差过小时重置积分
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
            ROS_INFO_THROTTLE(1, "[自动行驶] 剩余距离: %.2fm | 偏航误差: %.2f",
                             remaining_dist, yaw_error);
        } else if (current_state == ControlState::RETURN_HOME) {
            ROS_INFO_THROTTLE(1, "[返航中] 剩余距离: %.2fm | 偏航误差: %.2f",
                             remaining_dist, yaw_error);
        }
    }
}

// 初始化云台串口
bool init_yuntai_serial(const std::string& port, int baudrate) {
    try {
        sp.setPort(port);
        sp.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        sp.setTimeout(to);
        sp.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("无法打开云台串口 " << port << ": " << e.what());
        return false;
    }
    if (sp.isOpen()) {
        ROS_INFO_STREAM("云台串口 " << port << " 已打开 (波特率: " << baudrate << ")");
        return true;
    } else {
        ROS_ERROR_STREAM("云台串口 " << port << " 打开失败");
        return false;
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "integrated_car_gimbal_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 参数配置
    std::string yuntai_port;
    int yuntai_baud;
    pnh.param<std::string>("yuntai_port", yuntai_port, "/dev/ttyS6");
    pnh.param<int>("yuntai_baud", yuntai_baud, 115200);
    pnh.param<float>("linear_x", linear_x, 0.28f);
    pnh.param<float>("turn_speed", turn_speed, 0.5f);
    pnh.param<float>("target_distance", target_distance, 3.0f);
    pnh.param<int>("gimbal_up_pitch", gimbal_cfg.up_pitch, 90);  // 垂直向上角度
    pnh.param<float>("auto_return_delay", AUTO_RETURN_DELAY, 0.5f);

    // 初始化云台串口
    if (!init_yuntai_serial(yuntai_port, yuntai_baud)) {
        return -1;
    }

    // 初始化角度
    current_roll = gimbal_cfg.center_roll;
    current_pitch = gimbal_cfg.up_pitch;
    last_remote_input_time = ros::Time::now();

    // 发布订阅器初始化
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    yuntai_ret_info = nh.advertise<std_msgs::String>("/yuntai_return", 10);
    ros::Subscriber odom_sub = nh.subscribe("Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);

    // 启动自动回位定时器
    return_timer = nh.createTimer(ros::Duration(0.1), returnTimerCallback);

    ROS_INFO("小车与云台集成控制器启动成功!");
    ROS_INFO("云台朝上角度: %d度 | 自动回位延迟: %.1f秒", 
             gimbal_cfg.up_pitch, AUTO_RETURN_DELAY);
    ROS_INFO("云台控制通道: 水平(CH%d), 垂直(CH%d), 模式(CH%d)",
             gimbal_cfg.roll_channel + 1,
             gimbal_cfg.pitch_channel + 1,
             gimbal_cfg.mode_channel + 1);

    ros::spin();

    // 关闭资源
    if (sp.isOpen()) {
        yuntai_stop();
        sp.close();
    }
    return 0;
}
    





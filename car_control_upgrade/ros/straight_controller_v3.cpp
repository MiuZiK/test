/**
 * @file    straight_controller_v3.cpp
 * @brief   小车控制系统 v3 - 集成导航、安全、高级控制 (修复版)
 *
 * 升级内容:
 * 1. 多Waypoint导航 (PurePursuit算法)
 * 2. 安全看门狗系统
 * 3. 改进的PID控制器
 * 4. 状态监控与诊断
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <visualization_msgs/MarkerArray.h>

// ==================== 常量定义 ====================

const float DEG2RAD = M_PI / 180.0f;
const float RAD2DEG = 180.0f / M_PI;

// ==================== 控制状态机 ====================
enum class ControlState {
    STANDBY,
    ALIGN_DIRECTION,
    AUTO_STRAIGHT,
    WAIT_RETURN,
    RETURN_HOME,
    TURNING,
    WAYPOINT_NAV
};

enum class OperationMode {
    STANDBY_MODE,
    AUTO_MODE,
    MANUAL_MODE
};

enum class SafetyLevel : int {
    SAFE = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3
};

// ==================== Pure Pursuit 航向跟踪器 ====================
class PurePursuitController {
public:
    struct Params {
        float lookahead_base;
        float lookahead_gain;
        float max_angular_vel;
        float base_linear_speed;
        bool adaptive_lookahead;
    };

    PurePursuitController() : params_{0.8f, 1.5f, 0.6f, 0.33f, true} {}

    geometry_msgs::Twist compute(
        float robot_x, float robot_y, float robot_yaw,
        float target_x, float target_y,
        float current_speed = 0.3f
    ) {
        float dx = target_x - robot_x;
        float dy = target_y - robot_y;

        float local_x = dx * cos(-robot_yaw) - dy * sin(-robot_yaw);
        float local_y = dx * sin(-robot_yaw) + dy * cos(-robot_yaw);

        float lookahead = params_.lookahead_base;
        if (params_.adaptive_lookahead) {
            lookahead = params_.lookahead_base +
                        params_.lookahead_gain * fabs(current_speed);
            lookahead = std::max(0.3f, std::min(lookahead, 3.0f));
        }

        float dist_sq = local_x*local_x + local_y*local_y;
        if (dist_sq < 0.01f) dist_sq = 0.01f;

        float curvature = 2.0f * local_y / dist_sq;

        float max_curvature = params_.max_angular_vel / std::max(current_speed, 0.1f);
        curvature = std::max(std::min(curvature, max_curvature), -max_curvature);

        geometry_msgs::Twist cmd;
        cmd.linear.x = current_speed;
        cmd.angular.z = current_speed * curvature;

        return cmd;
    }

    void setParams(const Params& p) { params_ = p; }
    const Params& getParams() const { return params_; }

private:
    Params params_;
};

// ==================== Waypoint 管理器 ====================
class WaypointManager {
public:
    struct Waypoint {
        float x, y;
        float yaw;
        float tolerance;
        float max_speed;
        bool stop_at_point;

        Waypoint(float _x=0, float _y=0, float _yaw=NAN,
                float _tol=0.15f, float _speed=0.33f, bool _stop=false)
            : x(_x), y(_y), yaw(_yaw), tolerance(_tol),
              max_speed(_speed), stop_at_point(_stop) {}
    };

    void addWaypoint(float x, float y, float yaw = NAN,
                     float tol = 0.15f, float speed = 0.33f, bool stop = false) {
        waypoints_.push_back(Waypoint(x, y, yaw, tol, speed, stop));
    }

    void addWaypoint(const Waypoint& wp) {
        waypoints_.push_back(wp);
    }

    void clear() {
        waypoints_.clear();
        current_index_ = 0;
        completed_ = false;
    }

    size_t size() const { return waypoints_.size(); }
    bool isEmpty() const { return waypoints_.empty(); }
    bool isComplete() const { return completed_; }

    int getCurrentIndex() const { return current_index_; }
    int getRemainingCount() const { return (int)(waypoints_.size()) - current_index_; }

    Waypoint getCurrentWaypoint() const {
        if (current_index_ >= (int)waypoints_.size())
            return Waypoint(0, 0);
        return waypoints_[current_index_];
    }

    bool advanceToNext() {
        if (current_index_ >= (int)waypoints_.size() - 1) {
            completed_ = true;
            return false;
        }
        current_index_++;
        return true;
    }

    geometry_msgs::Twist computeCommand(
        const nav_msgs::Odometry& odom,
        PurePursuitController& pp_ctrl
    ) {
        if (isEmpty() || completed_) {
            geometry_msgs::Twist zero;
            return zero;
        }

        auto wp = getCurrentWaypoint();
        float cx = odom.pose.pose.position.x;
        float cy = odom.pose.pose.position.y;

        tf::Quaternion q;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
        double r, p, yaw;
        tf::Matrix3x3(q).getRPY(r, p, yaw);

        float dist = sqrt(pow(cx - wp.x, 2) + pow(cy - wp.y, 2));

        if (dist < wp.tolerance) {
            if (wp.stop_at_point || !advanceToNext()) {
                completed_ = true;
                geometry_msgs::Twist stop_cmd;
                return stop_cmd;
            }
            wp = getCurrentWaypoint();
        }

        auto cmd = pp_ctrl.compute(cx, cy, yaw, wp.x, wp.y, wp.max_speed);

        if (!std::isnan(wp.yaw) && dist < 1.0f) {
            float angle_err = shortestAngularDistance(yaw, wp.yaw);
            if (fabs(angle_err) > 0.1f) {
                cmd.linear.x *= 0.3f;
                cmd.angular.z += 0.3f * angle_err;
            }
        }

        return cmd;
    }

private:
    std::vector<Waypoint> waypoints_;
    int current_index_ = 0;
    bool completed_ = false;

    static float shortestAngularDistance(float from, float to) {
        float diff = to - from;
        while (diff > M_PI) diff -= 2*M_PI;
        while (diff < -M_PI) diff += 2*M_PI;
        return diff;
    }
};

// ==================== 安全看门狗系统 ====================
class SafetyWatchdog {
public:
    enum CheckItem {
        COMM_TIMEOUT,
        LOW_VOLTAGE,
        MOTOR_STALL,
        SPEED_ANOMALY,
        ANGLE_EXCESSIVE,
        ODOM_LOST
    };

    SafetyWatchdog()
        : comm_timeout_(0.5), voltage_warn_(10.8f), voltage_crit_(10.5f),
          stall_threshold_(0.02f), angle_limit_(30.0f * DEG2RAD),
          speed_ratio_max_(2.0f), safety_level_(SafetyLevel::SAFE)
    {}

    void setEmergencyCallback(std::function<void()> cb) {
        emergency_callback_ = cb;
    }

    void update(const ros::Time& now, const nav_msgs::Odometry& odom,
                 float battery_voltage, bool sbus_connected) {

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "SafetyWatchdog";
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "All systems normal";

        diagnostic_msgs::KeyValue kv;

        // 1. 通信检查
        double dt = (now - last_odom_time_).toSec();
        if (dt > comm_timeout_) {
            kv.key = "Comm Status"; kv.value = "TIMEOUT";
            status.values.push_back(kv);
            trigger(CheckItem::COMM_TIMEOUT, SafetyLevel::CRITICAL);
        } else {
            char buf[64];
            snprintf(buf, sizeof(buf), "DT=%.3fs", dt);
            kv.key = "Comm"; kv.value = buf;
            status.values.push_back(buf);
        }

        // 2. 电压检查
        if (battery_voltage > 0) {
            char vbuf[64];
            snprintf(vbuf, sizeof(vbuf), "%.2fV", battery_voltage);
            kv.key = "Voltage"; kv.value = vbuf;
            status.values.push_back(kv);

            if (battery_voltage < voltage_crit_) {
                trigger(CheckItem::LOW_VOLTAGE, SafetyLevel::EMERGENCY);
            } else if (battery_voltage < voltage_warn_) {
                trigger(CheckItem::LOW_VOLTAGE, SafetyLevel::WARNING);
            }
        }

        // 3. 速度异常检测
        float linear_vel = sqrt(
            pow(odom.twist.twist.linear.x, 2) +
            pow(odom.twist.twist.linear.y, 2)
        );
        if (target_speed_ > 0.1f && linear_vel > target_speed_ * speed_ratio_max_) {
            kv.key = "Speed"; kv.value = "ANOMALY";
            status.values.push_back(kv);
            trigger(CheckItem::SPEED_ANOMALY, SafetyLevel::WARNING);
        } else {
            char sbuf[64];
            snprintf(sbuf, sizeof(sbuf), "%.2f m/s", linear_vel);
            kv.key = "Speed"; kv.value = sbuf;
            status.values.push_back(kv);
        }

        safety_level_ = highest_level_;
        status.level = static_cast<uint8_t>(safety_level_);

        diag_msg_.status.clear();
        diag_msg_.status.push_back(status);
        diag_msg_.header.stamp = now;
    }

    SafetyLevel getSafetyLevel() const { return safety_level_; }
    bool isSafe() const { return safety_level_ <= SafetyLevel::WARNING; }
    bool isEmergency() const { return safety_level_ == SafetyLevel::EMERGENCY; }

    const diagnostic_msgs::DiagnosticArray& getDiagnostics() const { return diag_msg_; }

    void setTargetSpeed(float v) { target_speed_ = v; }

private:
    void trigger(CheckItem item, SafetyLevel level) {
        if ((int)level > (int)highest_level_) {
            highest_level_ = level;
        }
        fault_history_[item] = level;

        if (level == SafetyLevel::EMERGENCY && emergency_callback_) {
            emergency_callback_();
        }
    }

    float comm_timeout_, voltage_warn_, voltage_crit_;
    float stall_threshold_, angle_limit_, speed_ratio_max_;
    SafetyLevel safety_level_, highest_level_ = SafetyLevel::SAFE;
    ros::Time last_odom_time_;
    float target_speed_ = 0.0f;
    std::function<void()> emergency_callback_;
    std::map<CheckItem, SafetyLevel> fault_history_;
    diagnostic_msgs::DiagnosticArray diag_msg_;
};

// ==================== 主控制器 ====================
class UndercarriageInspectionControllerV3 {
public:
    UndercarriageInspectionControllerV3() : nh_("~") {}

    bool init() {
        // Parameters
        nh_.param<float>("linear_x", linear_x_, 0.33f);
        nh_.param<float>("turn_speed", turn_speed_, 0.5f);
        nh_.param<float>("kp", kp_, 1.5f);
        nh_.param<float>("ki", ki_, 0.08f);
        nh_.param<float>("kd", kd_, 0.3f);
        nh_.param<float>("feedforward", feedforward_, 0.04f);
        nh_.param<float>("target_distance", target_distance_, 16.0f);
        nh_.param<float>("target_tolerance", target_tolerance_, 0.15f);
        nh_.param<float>("angle_tolerance", angle_tolerance_, 0.05f);

        // Subscribers
        odom_sub_ = nh_.subscribe("/Odometry", 10,
            &UndercarriageInspectionControllerV3::odomCallback, this);
        sbus_sub_ = nh_.subscribe("/sbus", 10,
            &UndercarriageInspectionControllerV3::sbusCallback, this);

        // Publishers
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        gimbal_pub_ = nh_.advertise<std_msgs::String>("/ui_xy", 10);
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

        // Timer for gimbal
        gimbal_timer_ = nh_.createTimer(ros::Duration(0.1),
            &UndercarriageInspectionControllerV3::gimbalTimerCallback, this);

        // Safety callback
        watchdog_.setEmergencyCallback([this](){ this->emergencyStop(); });

        // Initialize gimbal
        publishGimbalCmd("0,90");

        printStartupInfo();

        last_callback_time_ = ros::Time::now();

        return true;
    }

    void run() {
        ros::spin();
    }

private:
    // === Callbacks ===
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        has_odom_ = true;
        ros::Time now = ros::Time::now();
        double dt = (now - last_callback_time_).toSec();
        last_callback_time_ = now;
        if (dt <= 0) dt = 0.01;

        watchdog_.update(now, msg, battery_voltage_, true);

        if (watchdog_.isEmergency()) {
            publishZeroVelocity();
            return;
        }

        switch (op_mode_) {
        case OperationMode::STANDBY_MODE:
            publishZeroVelocity();
            break;

        case OperationMode::MANUAL_MODE:
            if (remote_override_) {
                cmd_pub_.publish(remote_cmd_);
            } else {
                publishZeroVelocity();
            }
            break;

        case OperationMode::AUTO_MODE:
            handleAutoMode(msg, dt);
            break;
        }

        diag_pub_.publish(watchdog_.getDiagnostics());
    }

    void handleAutoMode(const nav_msgs::Odometry::ConstPtr& msg, float dt) {
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        double r, p, yaw;
        tf::Matrix3x3(q).getRPY(r, p, yaw);

        // INIT state
        if (current_state_ == ControlState::STANDBY ||
            current_state_ == ControlState::INIT) {
            home_x_ = x; home_y_ = y; home_yaw_ = yaw;
            target_direction_ = yaw;
            target_x_ = x + target_distance_ * cos(target_direction_);
            target_y_ = y + target_distance_ * sin(target_direction_);
            current_state_ = ControlState::ALIGN_DIRECTION;

            wp_manager_.clear();
            wp_manager_.addWaypoint(target_x_, target_y_, NAN, target_tolerance_, linear_x_, true);
            wp_manager_.addWaypoint(home_x_, home_y_, NAN, target_tolerance_, linear_x_*0.7f, true);

            ROS_INFO("[AUTO] Home:(%.2f,%.2f) Target:(%.2f,%.2f)", home_x_, home_y_, target_x_, target_y_);
            return;
        }

        auto cmd = wp_manager_.computeCommand(*msg, pp_ctrl_);

        if (wp_manager_.isComplete()) {
            if (current_state_ == ControlState::ALIGN_DIRECTION ||
                current_state_ == ControlState::AUTO_STRAIGHT) {
                current_state_ = ControlState::WAIT_RETURN;
                ROS_INFO("[AUTO] Reached target! Waiting for return command");
            } else if (current_state_ == ControlState::RETURN_HOME) {
                current_state_ = ControlState::WAIT_RETURN;
                ROS_INFO("[AUTO] Returned home safely!");
            }
            integral_error_ = 0;
            last_yaw_error_ = 0;
            publishZeroVelocity();
            return;
        }

        float remaining = calculateDistance(x, y, target_x_, target_y_);
        if (remaining > 0.5f && current_state_ != ControlState::RETURN_HOME) {
            float target_yaw = atan2(target_y_-y, target_x_-x);
            float yaw_err = shortestAngularDistance(yaw, target_yaw);
            float yaw_rate = (yaw_err - last_yaw_error_) / dt;
            last_yaw_error_ = yaw_err;

            integral_error_ += yaw_err * dt;
            integral_error_ = std::max(std::min(integral_error_, 1.0f), -1.0f);

            float pid_correction = kp_*yaw_err + ki_*integral_error_ + kd_*yaw_rate + feedforward_;
            pid_correction = std::max(std::min(pid_correction, MAX_ANGULAR_), -MAX_ANGULAR_);

            cmd.angular.z += pid_correction * 0.5f;
        }

        if (wp_manager_.getRemainingCount() == 1 && remaining < 1.0f) {
            cmd.linear.x *= pow(remaining, 2);
        }

        watchdog_.setTargetSpeed(cmd.linear.x);
        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(1, "[AUTO] WP[%d/%d] | Rem:%.2fm | Cmd:v=%.2f w=%.2f",
            wp_manager_.getCurrentIndex()+1, (int)wp_manager_.size(),
            remaining, cmd.linear.x, cmd.angular.z);
    }

    void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
        latest_sbus_ = *msg;
        prev_remote_override_ = remote_override_;

        uint16_t ch6 = msg->rawChannels[5];
        int ch6_state = (ch6 > 1500) ? 1 : ((ch6 < 1000) ? -1 : 0);

        if (ch6_state != prev_ch6_state_) {
            if (ch6_state == 1) {
                if (op_mode_ == OperationMode::STANDBY_MODE) enterAutoMode();
                else emergencyStop();
            } else if (ch6_state == -1) {
                emergencyStop();
            }
            prev_ch6_state_ = ch6_state;
        }

        bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
        bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

        if (ch1_active || ch2_active) {
            if (op_mode_ != OperationMode::MANUAL_MODE) enterManualMode();
            remote_override_ = true;
            remote_cmd_.angular.z = MAX_ANGULAR_ * (ch1_active?msg->mappedChannels[0]/100:0);
            remote_cmd_.linear.x = linear_x_ * (ch2_active?msg->mappedChannels[1]/100:0);
        } else {
            remote_override_ = false;
            if (prev_remote_override_ && op_mode_ == OperationMode::MANUAL_MODE) {
                if (has_odom_) enterAutoMode();
                else enterStandbyMode();
            }
        }

        if (op_mode_ == OperationMode::AUTO_MODE && !remote_override_) {
            uint16_t ch5 = msg->rawChannels[4];
            if (current_state_ == ControlState::WAIT_RETURN && ch5 > 1500) {
                current_state_ = ControlState::TURNING;
                ROS_INFO("[CH5] Return triggered!");
            }
        }
    }

    // === Mode transitions ===
    void enterStandbyMode() {
        op_mode_ = OperationMode::STANDBY_MODE;
        current_state_ = ControlState::STANDBY;
        resetPID();
        publishZeroVelocity();
        ROS_WARN("[MODE] >>> STANDBY <<<");
    }

    void enterAutoMode() {
        if (!has_odom_) { ROS_WARN("[MODE] No odometry"); return; }
        op_mode_ = OperationMode::AUTO_MODE;
        current_state_ = ControlState::INIT;
        resetPID();
        ROS_INFO("[MODE] >>> AUTO <<<");
    }

    void enterManualMode() {
        op_mode_ = OperationMode::MANUAL_MODE;
        ROS_INFO("[MODE] >>> MANUAL <<<");
    }

    void emergencyStop() {
        enterStandbyMode();
        ROS_ERROR("[EMERGENCY] STOP!");
    }

    void resetPID() {
        integral_error_ = 0;
        last_yaw_error_ = 0;
    }

    void publishZeroVelocity() {
        geometry_msgs::Twist z;
        cmd_pub_.publish(z);
    }

    void publishGimbalCmd(const std::string& cmd) {
        std_msgs::String m; m.data = cmd;
        gimbal_pub_.publish(m);
    }

    void gimbalTimerCallback(const ros::TimerEvent&) {
        publishGimbalCmd("0,90");
    }

    static float calculateDistance(float x1, float y1, float x2, float y2) {
        return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
    }

    static float shortestAngularDistance(float from, float to) {
        float d = to-from;
        while(d>M_PI)d-=2*M_PI; while(d<-M_PI)d+=2*M_PI;
        return d;
    }

    void printStartupInfo() {
        ROS_INFO("="*55);
        ROS_INFO(" Undercarriage Inspection Controller V3.0");
        ROS_INFO("="*55);
        ROS_INFO(" Features:");
        ROS_INFO("   + PurePursuit navigation");
        ROS_INFO("   + Multi-Waypoint support");
        ROS_INFO("   + Safety Watchdog system");
        ROS_INFO("   + Enhanced PID control");
        ROS_INFO("="*55);
        ROS_INFO(" CH6 UP   -> START | CH6 DOWN -> E-STOP");
        ROS_INFO(" CH1/CH2  -> MANUAL mode");
        ROS_INFO(" CH5      -> Return home");
        ROS_INFO("="*55);
    }

    // ==================== Members ====================
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, sbus_sub_;
    ros::Publisher cmd_pub_, gimbal_pub_, diag_pub_;
    ros::Timer gimbal_timer_;

    ControlState current_state_ = ControlState::STANDBY;
    OperationMode op_mode_ = OperationMode::STANDBY_MODE;
    nav_msgs::Odometry current_odom_;
    sbus_serial::Sbus latest_sbus_;
    bool has_odom_ = false, remote_override_ = false, prev_remote_override_ = false;
    ros::Time last_callback_time_;

    // Navigation
    PurePursuitController pp_ctrl_;
    WaypointManager wp_manager_;
    float home_x_=0, home_y_=0, home_yaw_=0;
    float target_x_=0, target_y_=0, target_direction_=0;

    // PID
    float linear_x_=0.33f, turn_speed_=0.5f;
    float kp_=1.5f, ki_=0.08f, kd_=0.3f, feedforward_=0.04f;
    float target_distance_=16.0f, target_tolerance_=0.15f, angle_tolerance_=0.05f;
    float integral_error_=0, last_yaw_error_=0;

    // Safety
    SafetyWatchdog watchdog_;
    float battery_voltage_ = 12.0f;

    // Constants
    const float STICK_DEADZONE = 15.0f;
    const float MAX_ANGULAR_ = 0.5f;
    int prev_ch6_state_ = 0;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "undercarriage_inspector_v3");

    UndercarriageInspectionControllerV3 controller;
    if (!controller.init()) return -1;

    controller.run();
    return 0;
}

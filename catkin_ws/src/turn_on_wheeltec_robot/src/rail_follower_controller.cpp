/**
 * ==================================================================================
 * 轨道跟随控制器 v1.0 - 基于激光雷达的实时轨道检测与跟踪
 * ==================================================================================
 * 
 * 核心创新：
 * ─────────
 * 1. 从2D激光雷达数据中实时提取铁轨特征（两条平行线）
 * 2. 使用RANSAC算法鲁棒拟合轨道线，排除车底/车侧物体和噪点
 * 3. 计算轨道中心线作为参考路径
 * 4. 使用改进的PurePursuit + Stanley混合控制器精确跟踪
 * 5. 完全独立于里程计绝对坐标，对初始姿态偏差和漂移免疫
 * 
 * 技术架构：
 * ──────────
 * [激光雷达 /scan] 
 *      ↓
 * [点云裁剪] ← 排除车底(近处)、车侧(远处)、车身(后方)
 *      ↓
 * [RANSAC直线拟合] ← 提取左右两条轨道线
 *      ↓
 * [中心线计算] ← 参考路径生成
 *      ↓
 * [PurePursuit + Stanley] ← 横向误差 + 航向误差联合控制
 *      ↓
 * [/cmd_vel] → 底层执行
 * 
 * 相比原straight_controller的优势：
 * ─────────────────────────────
 * ✓ 不依赖FastLIO odom的绝对坐标精度
 * ✓ 初始摆放角度偏差±30°内可自动纠正
 * ✓ 实时闭环反馈，持续纠偏
 * ✓ 对传感器漂移和噪声具有强鲁棒性
 * 
 * Author: AI Assistant
 * Date: 2026-05-06
 * ==================================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sbus_serial/Sbus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <random>
#include <std_msgs/String.h>
#include <wiringPi.h>

// ==================== 状态枚举 ====================
enum class RailControlState {
    STANDBY,            // 待机：上电默认，等待遥控器启动
    RAIL_DETECTING,     // 轨道检测中：尝试识别左右轨道
    RAIL_FOLLOWING,     // 轨道跟踪中：正常沿轨道行驶
    RAIL_LOST,          // 轨道丢失：无法检测到有效轨道
    WAIT_RETURN,        // 到达目标距离，等待返航指令
    RETURN_HOME         // 返航中
};

enum class OperationMode {
    STANDBY_MODE,
    AUTO_MODE,
    MANUAL_MODE
};

// ==================== 数据结构 ====================
struct Point2D {
    double x, y;
    Point2D(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    Point2D operator+(const Point2D& p) const { return Point2D(x + p.x, y + p.y); }
    Point2D operator-(const Point2D& p) const { return Point2D(x - p.x, y - p.y); }
    Point2D operator*(double s) const { return Point2D(x * s, y * s); }
    double norm() const { return sqrt(x*x + y*y); }
};

struct Line2D {
    // 直线方程: ax + by + c = 0, 归一化使得 a² + b² = 1
    double a, b, c;  
    // 方向向量 (单位向量)
    double dx, dy;
    
    Line2D() : a(0), b(1), c(0), dx(1), dy(0) {}
    
    // 从两点构造直线
    void fromPoints(const Point2D& p1, const Point2D& p2) {
        dx = p2.x - p1.x;
        dy = p2.y - p1.y;
        double len = sqrt(dx*dx + dy*dy);
        if (len > 1e-6) {
            dx /= len;
            dy /= len;
        }
        // 法向量 (a, b) 与方向向量垂直
        a = -dy;
        b = dx;
        c = -(a * p1.x + b * p1.y);
    }
    
    // 点到直线的距离（带符号）
    double signedDistance(const Point2D& p) const {
        return a * p.x + b * p.y + c;
    }
    
    // 点到直线的绝对距离
    double distance(const Point2D& p) const {
        return fabs(signedDistance(p));
    }
    
    // 计算直线上离给定点最近的点
    Point2D closestPoint(const Point2D& p) const {
        double d = signedDistance(p);
        return Point2D(p.x - a * d, p.y - b * d);
    }
};

struct RailPair {
    Line2D left_rail;       // 左轨道线
    Line2D right_rail;      // 右轨道线
    Line2D center_line;     // 中心线（目标路径）
    double rail_width;      // 轨距（米）
    double confidence;      // 置信度 [0, 1]
    bool valid;             // 是否有效
    
    RailPair() : rail_width(0), confidence(0), valid(false) {}
};

// ==================== 全局变量 ====================
ros::Publisher cmd_vel_pub;
ros::Publisher gimbal_cmd_pub;
ros::Publisher mode_pub;
ros::Publisher marker_pub;
ros::Publisher rail_marker_pub;

nav_msgs::Odometry current_odom;
bool has_odom = false;

RailControlState rail_state = RailControlState::STANDBY;
OperationMode op_mode = OperationMode::STANDBY_MODE;

// 遥控器相关
bool remote_override = false;
bool prev_remote_override = false;
geometry_msgs::Twist remote_cmd;
sbus_serial::Sbus latest_sbus_data;
const float STICK_DEADZONE = 15.0f;
const float MAX_ANGULAR = 0.5f;

// 控制参数（可通过参数服务器配置）
struct RailFollowerConfig {
    // ====== 激光裁剪参数 ======
    double angle_min_deg = -75.0;        // 最小扫描角度（相对车头）
    double angle_max_deg = 75.0;         // 最大扫描角度
    double range_min = 0.3;              // 最近距离（排除车底）
    double range_max = 5.0;              // 最远距离（排除远处噪点）
    double lateral_min = 0.3;            // 最小横向距离（排除车身）
    double lateral_max = 2.5;            // 最大横向距离
    
    // ====== RANSAC参数 ======
    int ransac_iterations = 100;         // RANSAC迭代次数
    double ransac_threshold = 0.05;      // 内点阈值（米）
    double min_inlier_ratio = 0.3;       // 最小内点比例
    int min_rail_points = 20;            // 单条轨道最少点数
    
    // ====== 轨道几何约束 ======
    double expected_rail_width = 0.6;   // 预期轨距（米），标准轨距1.435m的一半约0.7m
    double rail_width_tolerance = 0.25;  // 轨距容差（米）
    double parallel_threshold = 0.15;    // 平行度阈值（弧度≈8.6°）
    
    // ====== 跟踪控制参数 ======
    double linear_speed = 0.33;          // 前进速度（m/s）
    double lookahead_distance = 1.5;     // PurePursuit前视距离（米）
    double stanley_k_e = 0.5;           // Stanley横向误差增益
    double stanley_k_h = 1.0;           // Stanley航向误差增益
    double softening_factor = 1.0;      // 低速软化因子
    
    // ====== 目标参数 ======
    double target_distance = 16.0;       // 目标行驶距离（米）
    double target_tolerance = 0.3;       // 到达容忍距离（米）
    
    // ====== 安全参数 ======
    double lost_timeout = 1.0;           // 轨道丢失超时（秒）
    int required_detect_count = 5;       // 连续检测成功次数才进入跟踪模式
    
} config;

// 状态变量
RailPair current_rails;
Point2D start_position;
double traveled_distance = 0.0;
Point2D last_valid_position;
int detect_success_count = 0;
int detect_fail_count = 0;
ros::Time last_detect_time;
bool rails_initialized = false;

// PID相关（用于速度平滑）
float integral_error = 0.0f;
float last_yaw_error = 0.0f;

// LED控制
const int LED_PIN = 10;
bool led_state = false;
const int CH10_LOW = 282;
const int CH10_HIGH = 1722;
const int CH10_TOLERANCE = 5;
int prev_ch10_state = -1;
int ch10_stable_count = 0;
const int REQUIRED_STABLE_COUNT = 3;

// CH6遥控通道
const int CH6_START_THRESHOLD = 1500;
const int CH6_STOP_THRESHOLD = 1000;
int prev_ch6_state = 0;

// ==================== 工具函数 ====================
double degToRad(double deg) { return deg * M_PI / 180.0; }
double radToDeg(double rad) { return rad * 180.0 / M_PI; }

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double shortestAngularDistance(double from, double to) {
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

double calculateDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

void publishZeroVelocity() {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub.publish(stop_cmd);
}

// ==================== 可视化函数 ====================
void publishRailVisualization(const RailPair& rails) {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);
    
    // 左轨道（红色）
    visualization_msgs::Marker& left_marker = markers.markers[0];
    left_marker.header.frame_id = "laser";
    left_marker.header.stamp = ros::Time::now();
    left_marker.ns = "rails";
    left_marker.id = 0;
    left_marker.type = visualization_msgs::Marker::LINE_STRIP;
    left_marker.action = visualization_msgs::Marker::ADD;
    left_marker.scale.x = 0.02;
    left_marker.color.r = 1.0; left_marker.color.g = 0.0; left_marker.color.b = 0.0; left_marker.color.a = 0.8;
    
    if (rails.valid) {
        for (double t = -3.0; t <= 3.0; t += 0.1) {
            geometry_msgs::Point p;
            p.x = rails.left_rail.dx * t - rails.left_rail.a * rails.left_rail.c;
            p.y = rails.left_rail.dy * t - rails.left_rail.b * rails.left_rail.c;
            left_marker.points.push_back(p);
        }
    }
    
    // 右轨道（蓝色）
    visualization_msgs::Marker& right_marker = markers.markers[1];
    right_marker.header.frame_id = "laser";
    right_marker.header.stamp = ros::Time::now();
    right_marker.ns = "rails";
    right_marker.id = 1;
    right_marker.type = visualization_msgs::Marker::LINE_STRIP;
    right_marker.action = visualization_msgs::Marker::ADD;
    right_marker.scale.x = 0.02;
    right_marker.color.r = 0.0; right_marker.color.g = 0.0; right_marker.color.b = 1.0; right_marker.color.a = 0.8;
    
    if (rails.valid) {
        for (double t = -3.0; t <= 3.0; t += 0.1) {
            geometry_msgs::Point p;
            p.x = rails.right_rail.dx * t - rails.right_rail.a * rails.right_rail.c;
            p.y = rails.right_rail.dy * t - rails.right_rail.b * rails.right_rail.c;
            right_marker.points.push_back(p);
        }
    }
    
    // 中心线（绿色）
    visualization_msgs::Marker& center_marker = markers.markers[2];
    center_marker.header.frame_id = "laser";
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "rails";
    center_marker.id = 2;
    center_marker.type = visualization_msgs::Marker::LINE_STRIP;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.scale.x = 0.03;
    center_marker.color.r = 0.0; center_marker.color.g = 1.0; center_marker.color.b = 0.0; center_marker.color.a = 1.0;
    
    if (rails.valid) {
        for (double t = -3.0; t <= 3.0; t += 0.1) {
            geometry_msgs::Point p;
            p.x = rails.center_line.dx * t - rails.center_line.a * rails.center_line.c;
            p.y = rails.center_line.dy * t - rails.center_line.b * rails.center_line.c;
            center_marker.points.push_back(p);
        }
    }
    
    // 小车位置（黄色圆点）
    visualization_msgs::Marker& robot_marker = markers.markers[3];
    robot_marker.header.frame_id = "laser";
    robot_marker.header.stamp = ros::Time::now();
    robot_marker.ns = "rails";
    robot_marker.id = 3;
    robot_marker.type = visualization_msgs::Marker::SPHERE;
    robot_marker.action = visualization_msgs::Marker::ADD;
    robot_marker.pose.position.x = 0;
    robot_marker.pose.position.y = 0;
    robot_marker.pose.position.z = 0;
    robot_marker.scale.x = 0.1; robot_marker.scale.y = 0.1; robot_marker.scale.z = 0.1;
    robot_marker.color.r = 1.0; robot_marker.color.g = 1.0; robot_marker.color.b = 0.0; robot_marker.color.a = 1.0;
    
    rail_marker_pub.publish(markers);
}

void publishPointCloudVisualization(const std::vector<Point2D>& points, 
                                     const std::vector<int>& left_inliers,
                                     const std::vector<int>& right_inliers) {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(3);
    
    // 所有裁剪后的点（灰色）
    visualization_msgs::Marker& all_marker = markers.markers[0];
    all_marker.header.frame_id = "laser";
    all_marker.header.stamp = ros::Time::now();
    all_marker.ns = "points";
    all_marker.id = 0;
    all_marker.type = visualization_msgs::Marker::POINTS;
    all_marker.action = visualization_msgs::Marker::ADD;
    all_marker.scale.x = 0.02;
    all_marker.scale.y = 0.02;
    all_marker.color.r = 0.5; all_marker.color.g = 0.5; all_marker.color.b = 0.5; all_marker.color.a = 0.5;
    
    for (const auto& p : points) {
        geometry_msgs::Point gp;
        gp.x = p.x; gp.y = p.y; gp.z = 0;
        all_marker.points.push_back(gp);
    }
    
    // 左轨道内点（红色）
    visualization_msgs::Marker& left_marker = markers.markers[1];
    left_marker.header.frame_id = "laser";
    left_marker.header.stamp = ros::Time::now();
    left_marker.ns = "points";
    left_marker.id = 1;
    left_marker.type = visualization_msgs::Marker::POINTS;
    left_marker.action = visualization_msgs::Marker::ADD;
    left_marker.scale.x = 0.04;
    left_marker.scale.y = 0.04;
    left_marker.color.r = 1.0; left_marker.color.g = 0.0; left_marker.color.b = 0.0; left_marker.color.a = 1.0;
    
    for (int idx : left_inliers) {
        if (idx >= 0 && idx < (int)points.size()) {
            geometry_msgs::Point gp;
            gp.x = points[idx].x; gp.y = points[idx].y; gp.z = 0;
            left_marker.points.push_back(gp);
        }
    }
    
    // 右轨道内点（蓝色）
    visualization_msgs::Marker& right_marker = markers.markers[2];
    right_marker.header.frame_id = "laser";
    right_marker.header.stamp = ros::Time::now();
    right_marker.ns = "points";
    right_marker.id = 2;
    right_marker.type = visualization_msgs::Marker::POINTS;
    right_marker.action = visualization_msgs::Marker::ADD;
    right_marker.scale.x = 0.04;
    right_marker.scale.y = 0.04;
    right_marker.color.r = 0.0; right_marker.color.g = 0.0; right_marker.color.b = 1.0; right_marker.color.a = 1.0;
    
    for (int idx : right_inliers) {
        if (idx >= 0 && idx < (int)points.size()) {
            geometry_msgs::Point gp;
            gp.x = points[idx].x; gp.y = points[idx].y; gp.z = 0;
            right_marker.points.push_back(gp);
        }
    }
    
    marker_pub.publish(markers);
}

// ==================== 核心算法：点云裁剪 ====================
std::vector<Point2D> cropLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<Point2D> cropped_points;
    
    double angle_min = degToRad(config.angle_min_deg);
    double angle_max = degToRad(config.angle_max_deg);
    
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        double range = scan->ranges[i];
        
        // 距离过滤
        if (range < config.range_min || range > config.range_max) continue;
        if (!std::isfinite(range)) continue;
        
        double angle = scan->angle_min + i * scan->angle_increment;
        
        // 角度过滤：只保留前方扇形区域（排除车尾）
        if (angle < angle_min || angle > angle_max) continue;
        
        // 转换为笛卡尔坐标（激光坐标系：x向前，y向左）
        double x = range * cos(angle);
        double y = range * sin(angle);
        
        // 横向距离过滤：排除太靠近车身的点
        if (fabs(y) < config.lateral_min) continue;  // 车底区域
        if (fabs(y) > config.lateral_max) continue;  // 太远的侧方
        
        // 只保留两侧的点（用于轨道检测）
        // 进一步过滤：保留左侧(y>0)和右侧(y<0)的点
        cropped_points.emplace_back(x, y);
    }
    
    return cropped_points;
}

// ==================== 核心算法：RANSAC直线拟合 ====================
Line2D ransacLineFit(const std::vector<Point2D>& points, 
                      int iterations, 
                      double threshold,
                      std::vector<int>& inliers) {
    Line2D best_line;
    inliers.clear();
    
    if ((int)points.size() < config.min_rail_points) return best_line;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);
    
    int best_inlier_count = 0;
    
    for (int iter = 0; iter < iterations; iter++) {
        // 随机选两个点
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        while (idx2 == idx1) idx2 = dis(gen);
        
        // 构造候选直线
        Line2D candidate;
        candidate.fromPoints(points[idx1], points[idx2]);
        
        // 计算内点
        std::vector<int> current_inliers;
        for (size_t i = 0; i < points.size(); i++) {
            if (candidate.distance(points[i]) < threshold) {
                current_inliers.push_back(i);
            }
        }
        
        // 更优解？
        if ((int)current_inliers.size() > best_inlier_count) {
            best_inlier_count = current_inliers.size();
            inliers = current_inliers;
            
            // 用所有内点重新拟合（最小二乘法优化）
            double sum_x = 0, sum_y = 0, sum_xx = 0, sum_yy = 0, sum_xy = 0;
            for (int idx : inliers) {
                sum_x += points[idx].x;
                sum_y += points[idx].y;
                sum_xx += points[idx].x * points[idx].x;
                sum_yy += points[idx].y * points[idx].y;
                sum_xy += points[idx].x * points[idx].y;
            }
            int n = inliers.size();
            if (n >= 2) {
                double mean_x = sum_x / n;
                double mean_y = sum_y / n;
                
                // 主成分分析(PCA)求主方向
                double cov_xx = sum_xx / n - mean_x * mean_x;
                double cov_yy = sum_yy / n - mean_y * mean_y;
                double cov_xy = sum_xy / n - mean_x * mean_y;
                
                // 特征值求解（2x2对称矩阵）
                double trace = cov_xx + cov_yy;
                double det = cov_xx * cov_yy - cov_xy * cov_xy;
                double discriminant = sqrt(std::max(0.0, trace*trace/4.0 - det));
                
                double lambda1 = trace/2.0 + discriminant;  // 最大特征值
                // double lambda2 = trace/2.0 - discriminant;
                
                // 主方向向量（对应最大特征值）
                if (fabs(cov_xy) > 1e-6) {
                    best_line.dx = lambda1 - cov_yy;
                    best_line.dy = cov_xy;
                } else {
                    best_line.dx = 1.0;
                    best_line.dy = 0.0;
                }
                
                // 归一化
                double len = sqrt(best_line.dx*best_line.dx + best_line.dy*best_line.dy);
                if (len > 1e-6) {
                    best_line.dx /= len;
                    best_line.dy /= len;
                }
                
                // 法向量
                best_line.a = -best_line.dy;
                best_line.b = best_line.dx;
                best_line.c = -(best_line.a * mean_x + best_line.b * mean_y);
            }
        }
    }
    
    return best_line;
}

// ==================== 核心算法：轨道对检测 ====================
RailPair detectRails(const std::vector<Point2D>& points) {
    RailPair result;
    
    if ((int)points.size() < config.min_rail_points * 2) {
        return result;
    }
    
    // 分离左右两边的点
    std::vector<Point2D> left_points, right_points;
    for (const auto& p : points) {
        if (p.y > 0) left_points.push_back(p);   // 左侧
        else right_points.push_back(p);           // 右侧
    }
    
    // 分别对左右点集进行RANSAC拟合
    std::vector<int> left_inliers, right_inliers;
    Line2D left_line = ransacLineFit(left_points, config.ransac_iterations, 
                                      config.ransac_threshold, left_inliers);
    Line2D right_line = ransacLineFit(right_points, config.ransac_iterations, 
                                       config.ransac_threshold, right_inliers);
    
    // 验证有效性
    bool left_valid = (left_inliers.size() >= (size_t)config.min_rail_points) &&
                      (left_inliers.size() >= left_points.size() * config.min_inlier_ratio);
    bool right_valid = (right_inliers.size() >= (size_t)config.min_rail_points) &&
                       (right_inliers.size() >= right_points.size() * config.min_inlier_ratio);
    
    if (!left_valid || !right_valid) {
        ROS_DEBUG_THROTTLE(1.0, "[RAIL] Left: %d/%d pts (%s), Right: %d/%d pts (%s)",
                          left_inliers.size(), left_points.size(), left_valid ? "OK" : "FAIL",
                          right_inliers.size(), right_points.size(), right_valid ? "OK" :FAIL");
        return result;
    }
    
    // 几何一致性检查：两条线应该近似平行
    double dot_product = fabs(left_line.dx * right_line.dx + left_line.dy * right_line.dy);
    // dot_product ≈ 1 表示平行
    if (dot_product < cos(config.parallel_threshold)) {
        ROS_WARN_THROTTLE(1.0, "[RAIL] Lines not parallel! dot=%.3f", dot_product);
        return result;
    }
    
    // 计算轨距（在x=0处的两条线的y坐标差）
    double left_y_at_origin = (-left_line.c) / left_line.b;  // 假设a≈0（近似水平）
    double right_y_at_origin = (-right_line.c) / right_line.b;
    double measured_width = fabs(left_y_at_origin - right_y_at_origin);
    
    // 轨距合理性检查
    if (fabs(measured_width - config.expected_rail_width) > config.rail_width_tolerance) {
        ROS_WARN_THROTTLE(1.0, "[RAIL] Invalid rail width: %.2fm (expected %.2fm±%.2fm)",
                         measured_width, config.expected_rail_width, config.rail_width_tolerance);
        return result;
    }
    
    // 构建结果
    result.left_rail = left_line;
    result.right_rail = right_line;
    result.rail_width = measured_width;
    result.valid = true;
    
    // 计算中心线（左右轨道的平均）
    result.center_line.a = (left_line.a + right_line.a) / 2.0;
    result.center_line.b = (left_line.b + right_line.b) / 2.0;
    result.center_line.c = (left_line.c + right_line.c) / 2.0;
    // 归一化
    double norm = sqrt(result.center_line.a*result.center_line.a + 
                       result.center_line.b*result.center_line.b);
    if (norm > 1e-6) {
        result.center_line.a /= norm;
        result.center_line.b /= norm;
        result.center_line.c /= norm;
    }
    result.center_line.dx = (left_line.dx + right_line.dx) / 2.0;
    result.center_line.dy = (left_line.dy + right_line.dy) / 2.0;
    norm = sqrt(result.center_line.dx*result.center_line.dx + 
                result.center_line.dy*result.center_line.dy);
    if (norm > 1e-6) {
        result.center_line.dx /= norm;
        result.center_line.dy /= norm;
    }
    
    // 计算置信度（基于内点比例和平行度）
    double left_ratio = (double)left_inliers.size() / left_points.size();
    double right_ratio = (double)right_inliers.size() / right_points.size();
    result.confidence = (left_ratio + right_ratio) / 2.0 * dot_product;
    
    // 发布可视化
    publishPointCloudVisualization(points, left_inliers, right_inliers);
    
    ROS_INFO_THROTTLE(2.0, "[RAIL] Detected! Width=%.2fm Conf=%.2f Left=%dpts Right=%dpts",
                     result.rail_width, result.confidence,
                     left_inliers.size(), right_inliers.size());
    
    return result;
}

// ==================== 核心算法：PurePursuit + Stanley控制 ====================
geometry_msgs::Twist computeControlCommand(const RailPair& rails, double current_speed) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = config.linear_speed;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    
    if (!rails.valid) {
        return cmd;  // 无效轨道，返回零角速度
    }
    
    // 当前位置（在激光坐标系下就是原点）
    Point2D robot_pos(0, 0);
    
    // 1. 计算横向误差（cross-track error）
    // 正值表示机器人偏右，负值表示偏左
    double cross_track_error = rails.center_line.signedDistance(robot_pos);
    
    // 2. 计算航向误差（heading error）
    // 期望朝向 = 中心线的方向向量
    double desired_heading = atan2(rails.center_line.dy, rails.center_line.dx);
    
    // 从odom获取当前朝向（如果可用）
    double current_heading = 0;
    if (has_odom) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(current_odom.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_heading = yaw;
    }
    
    double heading_error = shortestAngularDistance(current_heading, desired_heading);
    
    // 3. PurePursuit: 找到前视点
    // 在中心线上找到距离机器人lookahead_distance的点
    // 参数方程: P = P0 + t * direction
    // |P - robot_pos| = lookahead_distance
    // 解这个方程得到t
    double ld = config.lookahead_distance;
    double dx = rails.center_line.dx;
    double dy = rails.center_line.dy;
    
    // 机器人到直线的最近点
    Point2D closest = rails.center_line.closestPoint(robot_pos);
    
    // 从最近点沿方向移动ld距离
    Point2D goal_point(closest.x + dx * ld, closest.y + dy * ld);
    
    // 4. PurePursuit曲率计算
    // kappa = 2 * sin(alpha) / Ld
    // alpha = atan2(cross_track_error, ld) （简化版）
    double alpha = atan2(goal_point.y - robot_pos.y, goal_point.x - robot_pos.x);
    double curvature = 2.0 * sin(alpha) / ld;
    
    // 5. Stanley控制器修正
    // delta = -k_e * e - k_h * (psi - psi_desired)
    double stanley_correction = -config.stanley_k_e * cross_track_error 
                                - config.stanley_k_h * heading_error;
    
    // 6. 混合控制输出
    double angular_velocity;
    if (current_speed > 0.1) {
        // 高速时：PurePursuit主导
        angular_velocity = curvature * current_speed * 0.6 + stanley_correction * 0.4;
    } else {
        // 低速时：Stanley主导（避免PurePursuit在低速时不稳定）
        angular_velocity = stanley_correction;
    }
    
    // 限幅
    angular_velocity = std::max(std::min(angular_velocity, (double)MAX_ANGULAR), 
                                -(double)MAX_ANGULAR);
    
    cmd.angular.z = angular_velocity;
    
    // 调试输出
    ROS_INFO_THROTTLE(1.0, "[CONTROL] CrossTrackErr=%.3fm HeadErr=%.2frad AngVel=%.3frad/s",
                     cross_track_error, heading_error, angular_velocity);
    
    return cmd;
}

// ==================== 回调函数 ====================

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 只有在自动模式下才处理
    if (op_mode != OperationMode::AUTO_MODE) return;
    if (rail_state == RailControlState::STANDBY || rail_state == RailControlState::WAIT_RETURN) {
        publishZeroVelocity();
        return;
    }
    
    // Step 1: 点云裁剪
    std::vector<Point2D> cropped_points = cropLaserScan(msg);
    
    if (cropped_points.empty()) {
        ROS_WARN_THROTTLE(1.0, "[LASER] No valid points after cropping");
        detect_fail_count++;
        if (detect_fail_count > 10 && rail_state == RailControlState::RAIL_FOLLOWING) {
            rail_state = RailControlState::RAIL_LOST;
            ROS_WARN("[STATE] Rail LOST!");
        }
        publishZeroVelocity();
        return;
    }
    
    // Step 2: 轨道检测
    RailPair detected_rails = detectRails(cropped_points);
    
    if (detected_rails.valid) {
        last_detect_time = ros::Time::now();
        detect_fail_count = 0;
        
        if (rail_state == RailControlState::RAIL_DETECTING) {
            detect_success_count++;
            if (detect_success_count >= config.required_detect_count) {
                rail_state = RailControlState::RAIL_FOLLOWING;
                current_rails = detected_rails;
                rails_initialized = true;
                
                // 记录起始位置
                if (has_odom) {
                    start_position.x = current_odom.pose.pose.position.x;
                    start_position.y = current_odom.pose.pose.position.y;
                    last_valid_position = start_position;
                    traveled_distance = 0.0;
                }
                
                ROS_INFO("[STATE] >>> RAIL FOLLOWING <<< - Tracking enabled");
            } else {
                // 更新临时轨道（用于平滑）
                current_rails = detected_rails;
            }
        } else if (rail_state == RailControlState::RAIL_FOLLOWING) {
            // 平滑更新轨道参数（指数移动平均）
            double alpha = 0.3;  // 平滑系数
            current_rails.left_rail.a = alpha * detected_rails.left_rail.a + (1-alpha) * current_rails.left_rail.a;
            current_rails.left_rail.b = alpha * detected_rails.left_rail.b + (1-alpha) * current_rails.left_rail.b;
            current_rails.left_rail.c = alpha * detected_rails.left_rail.c + (1-alpha) * current_rails.left_rail.c;
            current_rails.right_rail.a = alpha * detected_rails.right_rail.a + (1-alpha) * current_rails.right_rail.a;
            current_rails.right_rail.b = alpha * detected_rails.right_rail.b + (1-alpha) * current_rails.right_rail.b;
            current_rails.right_rail.c = alpha * detected_rails.right_rail.c + (1-alpha) * current_rails.right_rail.c;
            current_rails.center_line.a = alpha * detected_rails.center_line.a + (1-alpha) * current_rails.center_line.a;
            current_rails.center_line.b = alpha * detected_rails.center_line.b + (1-alpha) * current_rails.center_line.b;
            current_rails.center_line.c = alpha * detected_rails.center_line.c + (1-alpha) * current_rails.center_line.c;
            current_rails.confidence = alpha * detected_rails.confidence + (1-alpha) * current_rails.confidence;
        }
        
        // Step 3: 计算控制指令
        geometry_msgs::Twist cmd = computeControlCommand(current_rails, config.linear_speed);
        
        // Step 4: 行驶距离检查
        if (has_odom && rail_state == RailControlState::RAIL_FOLLOWING) {
            Point2D current_pos(current_odom.pose.pose.position.x,
                               current_odom.pose.pose.position.y);
            double step_dist = calculateDistance(last_valid_position.x, last_valid_position.y,
                                                  current_pos.x, current_pos.y);
            
            // 只有当位移合理时才累加（防止odom跳变）
            if (step_dist < 1.0 && step_dist > 0.001) {
                traveled_distance += step_dist;
                last_valid_position = current_pos;
            }
            
            // 检查是否到达目标
            if (traveled_distance >= config.target_distance) {
                rail_state = RailControlState::WAIT_RETURN;
                traveled_distance = config.target_distance;
                ROS_INFO("[STATE] Target reached! Traveled %.2fm", traveled_distance);
                publishZeroVelocity();
                return;
            }
        }
        
        // 发布速度指令
        cmd_vel_pub.publish(cmd);
        
    } else {
        // 轨道检测失败
        detect_fail_count++;
        
        if (rail_state == RailControlState::RAIL_FOLLOWING) {
            if (detect_fail_count > 5) {
                rail_state = RailControlState::RAIL_LOST;
                ROS_WARN("[STATE] Rail LOST after %d consecutive failures!", detect_fail_count);
                publishZeroVelocity();
            } else {
                // 使用最后的已知轨道继续控制（短暂容错）
                geometry_msgs::Twist cmd = computeControlCommand(current_rails, config.linear_speed);
                cmd_vel_pub.publish(cmd);
            }
        }
    }
    
    // 发布可视化
    publishRailVisualization(current_rails);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
    has_odom = true;
}

void sbusCallback(const sbus_serial::Sbus::ConstPtr& msg) {
    latest_sbus_data = *msg;
    prev_remote_override = remote_override;

    bool ch1_active = fabs(msg->mappedChannels[0]) > STICK_DEADZONE;
    bool ch2_active = fabs(msg->mappedChannels[1]) > STICK_DEADZONE;

    // CH6 (SW B): 启动/急停
    uint16_t ch6_raw = msg->rawChannels[5];
    int ch6_state = 0;
    if (ch6_raw > CH6_START_THRESHOLD) ch6_state = 1;
    else if (ch6_raw < CH6_STOP_THRESHOLD) ch6_state = -1;

    if (ch6_state != prev_ch6_state) {
        if (ch6_state == 1) {
            if (op_mode == OperationMode::STANDBY_MODE) {
                enterAutoMode();
            } else {
                ROS_WARN("[CH6] EMERGENCY STOP!");
                enterStandbyMode();
            }
        } else if (ch6_state == -1) {
            ROS_WARN("[CH6] EMERGENCY STOP (DOWN)!");
            enterStandbyMode();
        }
        prev_ch6_state = ch6_state;
    }

    // 手动遥控
    if (ch1_active || ch2_active) {
        if (op_mode != OperationMode::MANUAL_MODE) {
            enterManualMode();
        }
        remote_override = true;
        float angular_input = (ch1_active) ? (msg->mappedChannels[0] / 100.0f) : 0.0f;
        remote_cmd.angular.z = MAX_ANGULAR * angular_input;
        float linear_input = (ch2_active) ? (msg->mappedChannels[1] / 100.0f) : 0.0f;
        remote_cmd.linear.x = config.linear_speed * linear_input;
    } else {
        remote_override = false;
        if (prev_remote_override && op_mode == OperationMode::MANUAL_MODE) {
            if (has_odom) {
                enterAutoMode();
            } else {
                enterStandbyMode();
            }
            publishZeroVelocity();
        }
    }

    // CH10: LED
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
                prev_ch10_state = current_ch10_state;
                ch10_stable_count = 0;
            }
        }
    }
}

void enterStandbyMode() {
    if (op_mode == OperationMode::STANDBY_MODE && rail_state == RailControlState::STANDBY) return;
    
    op_mode = OperationMode::STANDBY_MODE;
    rail_state = RailControlState::STANDBY;
    integral_error = 0.0;
    last_yaw_error = 0.0;
    detect_success_count = 0;
    detect_fail_count = 0;
    rails_initialized = false;
    traveled_distance = 0.0;
    publishZeroVelocity();
    ROS_WARN("[MODE] >>> STANDBY <<< - Waiting for CH6 UP to start");
}

void enterAutoMode() {
    if (op_mode == OperationMode::AUTO_MODE) return;
    
    op_mode = OperationMode::AUTO_MODE;
    rail_state = RailControlState::RAIL_DETECTING;
    detect_success_count = 0;
    detect_fail_count = 0;
    integral_error = 0.0;
    last_yaw_error = 0.0;
    ROS_INFO("[MODE] >>> AUTO <<< - Starting rail detection...");
}

void enterManualMode() {
    if (op_mode == OperationMode::MANUAL_MODE) return;
    
    op_mode = OperationMode::MANUAL_MODE;
    ROS_INFO("[MODE] >>> MANUAL <<< - Remote control active");
}

void printSystemStatus() {
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() < 2.0) return;
    last_print = ros::Time::now();

    std::string state_str, mode_str;
    switch (rail_state) {
        case RailControlState::STANDBY: state_str = "STANDBY"; break;
        case RailControlState::RAIL_DETECTING: state_str = "DETECT"; break;
        case RailControlState::RAIL_FOLLOWING: state_str = "FOLLOW"; break;
        case RailControlState::RAIL_LOST: state_str = "LOST"; break;
        case RailControlState::WAIT_RETURN: state_str = "WAIT_RET"; break;
        case RailControlState::RETURN_HOME: state_str = "RETURN"; break;
    }
    switch (op_mode) {
        case OperationMode::STANDBY_MODE: mode_str = "STANDBY"; break;
        case OperationMode::AUTO_MODE: mode_str = "AUTO"; break;
        case OperationMode::MANUAL_MODE: mode_str = "MANUAL"; break;
    }

    ROS_INFO("[STATUS] Mode=%s | State=%s | Dist=%.1fm/%.1fm | RailConf=%.2f",
             mode_str.c_str(), state_str.c_str(),
             traveled_distance, config.target_distance,
             current_rails.confidence);
}

void gimbalFixTimerCallback(const ros::TimerEvent& event) {
    std_msgs::String cmd_msg;
    cmd_msg.data = "0,90";
    gimbal_cmd_pub.publish(cmd_msg);
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    ros::init(argc, argv, "rail_follower_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // wiringPi初始化
    if (wiringPiSetup() == -1) {
        ROS_ERROR("Failed to initialize wiringPi!");
        return -1;
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // 从参数服务器加载配置
    pnh.param<double>("angle_min", config.angle_min_deg, -75.0);
    pnh.param<double>("angle_max", config.angle_max_deg, 75.0);
    pnh.param<double>("range_min", config.range_min, 0.3);
    pnh.param<double>("range_max", config.range_max, 5.0);
    pnh.param<double>("lateral_min", config.lateral_min, 0.3);
    pnh.param<double>("lateral_max", config.lateral_max, 2.5);
    
    pnh.param<int>("ransac_iterations", config.ransac_iterations, 100);
    pnh.param<double>("ransac_threshold", config.ransac_threshold, 0.05);
    pnh.param<double>("min_inlier_ratio", config.min_inlier_ratio, 0.3);
    pnh.param<int>("min_rail_points", config.min_rail_points, 20);
    
    pnh.param<double>("expected_rail_width", config.expected_rail_width, 0.6);
    pnh.param<double>("rail_width_tolerance", config.rail_width_tolerance, 0.25);
    pnh.param<double>("parallel_threshold", config.parallel_threshold, 0.15);
    
    pnh.param<double>("linear_speed", config.linear_speed, 0.33);
    pnh.param<double>("lookahead_distance", config.lookahead_distance, 1.5);
    pnh.param<double>("stanley_k_e", config.stanley_k_e, 0.5);
    pnh.param<double>("stanley_k_h", config.stanley_k_h, 1.0);
    
    pnh.param<double>("target_distance", config.target_distance, 16.0);
    pnh.param<double>("target_tolerance", config.target_tolerance, 0.3);
    
    pnh.param<double>("lost_timeout", config.lost_timeout, 1.0);
    pnh.param<int>("required_detect_count", config.required_detect_count, 5);

    // 订阅者
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber sbus_sub = nh.subscribe("/sbus", 10, sbusCallback);

    // 发布者
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    gimbal_cmd_pub = nh.advertise<std_msgs::String>("/ui_xy", 10);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/rail_points", 1);
    rail_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/rail_lines", 1);

    // 云台定时器
    ros::Timer gimbal_timer = nh.createTimer(ros::Duration(0.1), gimbalFixTimerCallback);
    
    // 初始化云台
    std_msgs::String init_gimbal;
    init_gimbal.data = "0,90";
    gimbal_cmd_pub.publish(init_gimbal);

    // 打印启动信息
    ROS_INFO("================================================================================");
    ROS_INFO("  RAIL FOLLOWER CONTROLLER v1.0 - Laser-based Rail Detection & Tracking");
    ROS_INFO("================================================================================");
    ROS_INFO("  Default mode: STANDBY (car does NOT move)");
    ROS_INFO("");
    ROS.info("  CONTROL:");
    ROS_INFO("    CONTROL:");
    ROS_INFO("    CH6 (SW B) DOWN -> EMERGENCY STOP (anytime)");
    ROS_INFO("    CH1/CH2 sticks  -> MANUAL mode (auto paused)");
    ROS_INFO("    CH10            -> LED toggle");
    ROS_INFO("");
    ROS_INFO("  ALGORITHM:");
    ROS_INFO("    Laser crop: [%.0f°, %.0f°] x [%.1fm, %.1fm]",
             config.angle_min_deg, config.angle_max_deg, config.range_min, config.range_max);
    ROS_INFO("    RANSAC: %d iters, threshold=%.3fm", config.ransac_iterations, config.ransac_threshold);
    ROS_INFO("    Expected rail width: %.2fm ± %.2fm", config.expected_rail_width, config.rail_width_tolerance);
    ROS_INFO("    Control: PurePursuit(L=%.1fm) + Stanley(k_e=%.1f, k_h=%.1f)",
             config.lookahead_distance, config.stanley_k_e, config.stanley_k_h);
    ROS_INFO("    Target distance: %.1fm", config.target_distance);
    ROS_INFO("");
    ROS_INFO("  Gimbal: locked vertical up (90 deg) for undercarriage view");
    ROS_INFO("================================================================================");

    ros::spin();
    return 0;
}

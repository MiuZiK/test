/**
 * @file    pid_controller.h
 * @brief   PID++ 增强型PID控制器 - 支持前馈、抗积分饱和、变化率限制、自适应
 * @version v2.0
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ==================== 配置宏 ==================== */
#ifndef PID_ABS
    #define PID_ABS(x)   ((x) >= 0 ? (x) : -(x))
#endif

#ifndef PID_CONSTRAIN
    #define PID_CONSTRAIN(x, min, max) \
        ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

/* ==================== 数据结构 ==================== */

/** PID控制器实例 */
typedef struct {
    /* --- 可调参数 --- */
    float Kp;              /* 比例增益 */
    float Ki;              /* 积分增益 */
    float Kd;              /* 微分增益 */
    float Kf;              /* 前馈增益 (基于设定值) */

    /* --- 限制参数 --- */
    float integral_max;     /* 积分累积最大值 (抗积分饱和) */
    float output_max;       /* 输出上限 */
    float output_min;       /* 输出下限 (-output_max if symmetric) */
    float rate_limit;       /* 输出变化率限制 (单位/周期) */
    float deadzone;         /* 死区阈值 */

    /* --- 内部状态 --- */
    float integral;         /* 积分累积值 */
    float prev_error;       /* 上一次误差 */
    float prev_input;       /* 上一次测量值 (用于微分onMeasurement) */
    float prev_output;      /* 上一次输出值 */
    float derivative;       /* 微分项 (带滤波) */
    float target;           /* 当前目标值 */

    /* --- 高级功能标志 --- */
    bool  derivative_on_measurement;  /* true=微分作用于测量值(推荐), false=微分作用于误差 */
    bool  enable_adaptive;            /* 启用自适应PID */
    bool  enable_feedforward;         /* 启用前馈补偿 */
    bool  enable_rate_limit;          /* 启用输出变化率限制 */
    bool  initialized;                /* 是否已初始化 */

} PID_t;

/** 自适应PID配置 */
typedef struct {
    float error_large_threshold;      /* 大误差阈值 */
    float error_small_threshold;      /* 小误差阈值 */
    float kp_large_factor;            /* 大误差时Kp倍率 */
    float ki_small_factor;            /* 小误差时Ki倍率 */
    float kd_normal_factor;           /* 正常Kd倍率 */
} AdaptiveConfig_t;

/* ==================== API 函数 ==================== */

/**
 * @brief 初始化PID控制器 (使用默认参数)
 * @param pid PID实例指针
 */
void PID_Init(PID_t* pid);

/**
 * @brief 使用指定参数初始化PID
 * @param pid PID实例指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void PID_InitWithParams(PID_t* pid, float kp, float ki, float kd);

/**
 * @brief 设置完整PID参数集
 * @param pid PID实例指针
 * @param Kp, Ki, Kd, Kf 各增益
 * @param integral_max 积分限幅
 * @param output_max 输出限幅
 * @param deadzone 死区
 * @param rate_limit 变化率限制
 */
void PID_SetFullParams(
    PID_t* pid,
    float Kp, float Ki, float Kd, float Kf,
    float integral_max, float output_max,
    float deadzone, float rate_limit
);

/**
 * @brief PID更新计算 - 主函数
 * @param pid PID实例指针
 * @param setpoint 目标值
 * @param measurement 测量值/反馈值
 * @param dt 时间间隔 (秒)
 * @return 控制输出
 *
 * 算法: output = P + I + D + F
 *   P = Kp * error
 *   I = Ki * integral(error)
 *   D = Kd * d(measurement)/dt 或 d(error)/dt
 *   F = Kf * setpoint (前馈)
 */
float PID_Update(PID_t* pid, float setpoint, float measurement, float dt);

/**
 * @brief 重置PID内部状态 (不改变参数)
 * @param pid PID实例指针
 */
void PID_Reset(PID_t* pid);

/**
 * @brief 设置新的目标值
 * @param pid PID实例指针
 * @param sp 新的目标值
 */
void PID_SetTarget(PID_t* pid, float sp);

/**
 * @brief 获取当前各项分量 (调试用)
 * @param pid PID实例指针
 * @param p_out 输出P分量
 * @param i_out 输出I分量
 * @param d_out 输出D分量
 * @param f_out 输出F分量
 */
void PID_GetComponents(const PID_t* pid, float* p_out, float* i_out, float* d_out, float* f_out);

/**
 * @brief 获取当前误差
 */
float PID_GetError(const PID_t* pid);

/**
 * @brief 自适应PID调参 (根据误差大小自动调整)
 * @param pid PID实例指针
 * @param config 自适应配置
 */
void PID_AdaptiveTune(PID_t* pid, const AdaptiveConfig_t* config);

/**
 * @brief 检查PID是否处于稳定状态
 * @param pid PID实例指针
 * @param threshold 稳定判定阈值
 * @return true=稳定 false=仍在调节中
 */
bool PID_IsStable(const PID_t* pid, float threshold);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */

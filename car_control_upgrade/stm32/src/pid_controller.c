/**
 * @file    pid_controller.c
 * @brief   PID++ 增强型PID控制器实现
 */

#include "pid_controller.h"

/* ==================== 默认参数 ==================== */
#define PID_DEFAULT_KP            1.0f
#define PID_DEFAULT_KI            0.1f
#define PID_DEFAULT_KD            0.05f
#define PID_DEFAULT_KF            0.5f
#define PID_DEFAULT_INTEGRAL_MAX  5000.0f
#define PID_DEFAULT_OUTPUT_MAX    16700.0f
#define PID_DEFAULT_DEADZONE      0.02f
#define PID_DEFAULT_RATE_LIMIT    2000.0f
#define PID_DT_MIN                0.001f  /* 最小有效dt */
#define PID_DERIVATIVE_LPF_ALPHA   0.8f    /* 微分低通滤波系数 (新值权重) */

/* ==================== 实现 ==================== */

void PID_Init(PID_t* pid) {
    if (pid == NULL) return;

    PID_InitWithParams(pid,
        PID_DEFAULT_KP,
        PID_DEFAULT_KI,
        PID_DEFAULT_KD);

    pid->Kf              = PID_DEFAULT_KF;
    pid->integral_max    = PID_DEFAULT_INTEGRAL_MAX;
    pid->output_max      = PID_DEFAULT_OUTPUT_MAX;
    pid->output_min      = -PID_DEFAULT_OUTPUT_MAX;
    pid->deadzone        = PID_DEFAULT_DEADZONE;
    pid->rate_limit      = PID_DEFAULT_RATE_LIMIT;

    pid->derivative_on_measurement = true;  /* 推荐设置 */
    pid->enable_adaptive             = false;
    pid->enable_feedforward          = true;
    pid->enable_rate_limit           = true;

    pid->initialized     = true;
    PID_Reset(pid);
}

void PID_InitWithParams(PID_t* pid, float kp, float ki, float kd) {
    if (pid == NULL) return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* 使用默认限制参数 */
    pid->integral_max    = PID_DEFAULT_INTEGRAL_MAX;
    pid->output_max      = PID_DEFAULT_OUTPUT_MAX;
    pid->output_min      = -PID_DEFAULT_OUTPUT_MAX;
    pid->deadzone        = PID_DEFAULT_DEADZONE;
    pid->rate_limit      = PID_DEFAULT_RATE_LIMIT;
    pid->Kf              = PID_DEFAULT_KF;

    pid->derivative_on_measurement = true;
    pid->initialized     = true;

    PID_Reset(pid);
}

void PID_SetFullParams(
    PID_t* pid, float Kp, float Ki, float Kd, float Kf,
    float integral_max, float output_max,
    float deadzone, float rate_limit
) {
    if (pid == NULL) return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->output_min = -output_max;
    pid->deadzone = deadzone;
    pid->rate_limit = rate_limit;
}

float PID_Update(PID_t* pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || !pid->initialized) return 0.0f;

    /* 保护: 防止dt过小或异常 */
    if (dt < PID_DT_MIN || dt > 10.0f) {
        return pid->prev_output;
    }

    pid->target = setpoint;

    /* 计算误差 */
    float error = setpoint - measurement;

    /* 死区处理 */
    if (pid->deadzone > 0 && PID_ABS(error) < pid->deadzone) {
        error = 0.0f;
    }

    /* ===== P项: 比例 ===== */
    float P = pid->Kp * error;

    /* ===== I项: 积分 (带抗积分饱和) ===== */
    /* 条件积分: 仅在误差不大时累积，避免超调 */
    bool integrate = true;
    if (pid->integral_max > 0) {
        /* 如果输出已经饱和且误差同向，则停止积分 */
        if ((pid->prev_output >= pid->output_max && error > 0) ||
            (pid->prev_output <= pid->output_min && error < 0)) {
            integrate = false;
        }
    }

    if (integrate) {
        pid->integral += error * dt;

        /* 积分限幅 */
        if (pid->integral_max > 0) {
            pid->integral = PID_CONSTRAIN(
                pid->integral,
                -pid->integral_max,
                pid->integral_max
            );
        }
    }

    float I = pid->Ki * pid->integral;

    /* ===== D项: 微分 ===== */
    float D = 0.0f;

    if (pid->derivative_on_measurement) {
        /* 微分作用于测量值 (推荐: 设定值突变时不会产生尖峰) */
        float d_meas = (measurement - pid->prev_input) / dt;
        /* 低通滤波平滑微分 */
        pid->derivative = PID_DERIVATIVE_LPF_ALPHA * d_meas +
                          (1.0f - PID_DERIVATIVE_LPF_ALPHA) * pid->derivative;
        D = -pid->Kd * pid->derivative;  /* 负号因为测量值变化方向与控制相反 */
    } else {
        /* 微分作用于误差 */
        float d_error = (error - pid->prev_error) / dt;
        pid->derivative = PID_DERIVATIVE_LPF_ALPHA * d_error +
                          (1.0f - PID_DERIVATIVE_LPF_ALPHA) * pid->derivative;
        D = pid->Kd * pid->derivative;
    }

    /* ===== F项: 前馈 ===== */
    float F = 0.0f;
    if (pid->enable_feedforward && pid->Kf != 0) {
        F = pid->Kf * setpoint;
    }

    /* ===== 组合输出 ===== */
    float output = P + I + D + F;

    /* 输出限幅 */
    output = PID_CONSTRAIN(output, pid->output_min, pid->output_max);

    /* 变化率限制 */
    if (pid->enable_rate_limit && pid->rate_limit > 0) {
        float delta = output - pid->prev_output;
        float max_delta = pid->rate_limit * dt;
        if (delta > max_delta) {
            output = pid->prev_output + max_delta;
        } else if (delta < -max_delta) {
            output = pid->prev_output - max_delta;
        }
    }

    /* 更新状态 */
    pid->prev_error  = error;
    pid->prev_input  = measurement;
    pid->prev_output = output;

    return output;
}

void PID_Reset(PID_t* pid) {
    if (pid == NULL) return;

    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->prev_input   = 0.0f;
    pid->prev_output  = 0.0f;
    pid->derivative   = 0.0f;
    pid->target       = 0.0f;
}

void PID_SetTarget(PID_t* pid, float sp) {
    if (pid == NULL) return;
    pid->target = sp;
}

void PID_GetComponents(const PID_t* pid, float* p_out, float* i_out, float* d_out, float* f_out) {
    if (pid == NULL) return;

    if (p_out) *p_out = pid->Kp * pid->prev_error;
    if (i_out) *i_out = pid->Ki * pid->integral;
    if (d_out) *d_out = -pid->Kd * pid->derivative;  /* 注意符号 */
    if (f_out) *f_out = pid->Kf * pid->target;
}

float PID_GetError(const PID_t* pid) {
    if (pid == NULL) return 0.0f;
    return pid->prev_error;
}

void PID_AdaptiveTune(PID_t* pid, const AdaptiveConfig_t* config) {
    if (pid == NULL || config == NULL || !pid->enable_adaptive) return;

    float error_mag = PID_ABS(pid->prev_error);
    float base_kp = pid->Kp / (
        pid->prev_error > config->error_large_threshold ? config->kp_large_factor :
        pid->prev_error < config->error_small_threshold ? 1.0f : 1.0f
    );

    /* 简化版自适应: 根据误差大小调整增益倍率 */
    if (error_mag > config->error_large_threshold) {
        /* 大误差: 增大P加快响应 */
        /* 实际应用中可以通过临时修改Kp实现，这里仅记录逻辑 */
    } else if (error_mag < config->error_small_threshold) {
        /* 小误差: 增大I消除稳态误差 */
    }
}

bool PID_IsStable(const PID_t* pid, float threshold) {
    if (pid == NULL) return true;
    return PID_ABS(pid->prev_error) < threshold &&
           PID_ABS(pid->integral) < threshold * 10;
}

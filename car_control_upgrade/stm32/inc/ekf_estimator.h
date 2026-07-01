/**
 * @file    ekf_estimator.h
 * @brief   扩展卡尔曼滤波状态估计器 - 融合编码器+IMU
 * 
 * 状态向量 (6维): [x, y, theta, v, omega, bias_omega]
 * 观测量 (4维): [v_left_enc, v_right_enc, gyro_z, accel_x]
 */

#ifndef __EKF_ESTIMATOR_H
#define __EKF_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ==================== 配置 ==================== */
#define EKF_STATE_DIM     6
#define EKF_MEAS_DIM      4
#define EKF_CONTROL_DIM   2  /* [a_linear, a_angular] */

typedef struct {
    /* 状态: x=[pos_x, pos_y, theta, v_linear, v_angular, gyro_bias] */
    float x[EKF_STATE_DIM];

    /* 协方差矩阵 P (6x6) */
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    /* 过程噪声协方差 Q */
    float Q[EKF_STATE_DIM][EKF_STATE_DIM];

    /* 测量噪声协方差 R */
    float R[EKF_MEAS_DIM][EKF_MEAS_DIM];

    /* 系统参数 */
    float wheel_base;           /* 轮距 m */
    float wheel_perimeter;      /* 轮周长 m */
    float encoder_ticks_per_rev; /* 编码器每圈脉冲数 */

    /* 时间戳 */
    uint32_t last_update_ms;
    bool initialized;

} EKF_t;

/** 测量值结构 */
typedef struct {
    float enc_left;             /* 左轮编码器速度 (m/s) */
    float enc_right;            /* 右轮编码器速度 (m/s) */
    float gyro_z;               /* Z轴角速度 (rad/s) from IMU */
    float accel_x;              /* X轴加速度 (m/s^2) from IMU */
} EKF_Measurement_t;

/** 控制输入 */
typedef struct {
    float cmd_v;                 /* 命令线速度 */
    float cmd_omega;             /* 命令角速度 */
} EKF_Control_t;

/** 估计结果 */
typedef struct {
    float x, y, theta;          /* 位姿 */
    float vx, vy;              /* 速度 (世界坐标系) */
    float v_linear;             /* 线速度 (车体坐标系) */
    float v_angular;            /* 角速度 */
    float confidence;           /* 置信度 0-1 */
} EKF_Estimate_t;

/* ==================== API ==================== */

void EKF_Init(EKF_t* ekf, float wheel_base);
void EKF_Reset(EKF_t* ekf, float x0, float y0, float theta0);

void EKF_Predict(EKF_t* ekf, const EKF_Control_t* u, float dt);
void EKF_Update(EKF_t* ekf, const EKF_Measurement_t* z);
void EKF_GetEstimate(const EKF_t* ekf, EKF_Estimate_t* est);

void EKF_SetProcessNoise(EKF_t* ekf, float q_pos, float q_vel, float q_bias);
void EKF_SetMeasurementNoise(EKF_t* ekf, float r_enc, float r_gyro, float r_accel);

#ifdef __cplusplus
}
#endif

#endif

/**
 * @file    ekf_estimator.c
 * @brief   EKF 实现 (简化版 - 适合嵌入式)
 */

#include "ekf_estimator.h"
#include <string.h>

static inline float wrap_angle(float a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

void EKF_Init(EKF_t* ekf, float wheel_base) {
    memset(ekf, 0, sizeof(EKF_t));
    ekf->wheel_base = wheel_base;
    ekf->wheel_perimeter = 0.32f;
    ekf->encoder_ticks_per_rev = 25 * 30; /* 减速比*编码器线数 */
    
    /* 初始协方差: 较大表示不确定 */
    for (int i=0; i<EKF_STATE_DIM; i++)
        ekf->P[i][i] = (i<3) ? 1.0f : 0.5f;

    /* 过程噪声 (默认值) */
    EKF_SetProcessNoise(ekf, 0.01f, 0.05f, 0.001f);
    EKF_SetMeasurementNoise(ekf, 0.1f, 0.02f, 0.5f);
}

void EKF_Reset(EKF_t* ekf, float x0, float y0, float theta0) {
    ekf->x[0] = x0;
    ekf->x[1] = y0;
    ekf->x[2] = theta0;
    ekf->x[3] = 0.0f;  /* v */
    ekf->x[4] = 0.0f;  /* omega */
    ekf->x[5] = 0.0f;  /* bias */

    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_STATE_DIM; j++) {
            ekf->P[i][j] = (i==j) ? ((i<3)?1.0f:0.5f) : 0.0f;
        }
    }
    ekf->initialized = true;
}

void EKF_Predict(EKF_t* ekf, const EKF_Control_t* u, float dt) {
    if (!ekf->initialized || dt <= 0 || dt > 1.0) return;

    float* x = ekf->x;
    const float th = x[2];
    const float v = x[3];
    const float w = x[4];

    /* 状态预测 (恒速模型 + 控制输入) */
    float cmd_v = u ? u->cmd_v : 0;
    float cmd_w = u ? u->cmd_omega : 0;

    /* 使用控制输入作为模型输入，状态作为修正 */
    float v_pred = (cmd_v != 0) ? cmd_v : v;
    float w_pred = (cmd_w != 0) ? cmd_w : w;

    /* 运动学预测 */
    if (fabs(w_pred) < 1e-6f) {
        x[0] += v_pred * cos(th) * dt;
        x[1] += v_pred * sin(th) * dt;
        x[2] = th;
    } else {
        x[0] += v_pred / w_pred * (sin(th + w_pred*dt) - sin(th));
        x[1] += v_pred / w_pred * (-cos(th + w_pred*dt) + cos(th));
        x[2] = wrap_angle(th + w_pred * dt);
    }
    x[3] = v_pred;
    x[4] = w_pred;
    /* gyro_bias 保持不变 (随机游走) */

    /* 雅可比矩阵 F (6x6) */
    float F[EKF_STATE_DIM][EKF_STATE_DIM];
    memset(F, 0, sizeof(F));

    F[0][0]=1; F[0][2]=-v_pred*sin(th)*dt; F[0][3]=cos(th)*dt;
    F[1][1]=1; F[1][2]= v_pred*cos(th)*dt; F[1][3]=sin(th)*dt;
    F[2][2]=1; F[2][4]=dt;
    F[3][3]=1;
    F[4][4]=1;
    F[5][5]=1;

    /* P_pred = F * P * F' + Q */
    float Pt[EKF_STATE_DIM][EKF_STATE_DIM];
    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_STATE_DIM; j++) {
            float sum = 0;
            for (int k=0; k<EKF_STATE_DIM; k++)
                sum += F[i][k] * ekf->P[k][j];  /* F*P */
            Pt[i][j] = sum;
        }
    }

    float Pnew[EKF_STATE_DIM][EKF_STATE_DIM];
    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_STATE_DIM; j++) {
            float sum = 0;
            for (int k=0; k<EKF_STATE_DIM; k++)
                sum += Pt[i][k] * F[j][k];  /* (F*P)*F' */
            Pnew[i][j] = sum + ekf->Q[i][j];
        }
    }
    memcpy(ekf->P, Pnew, sizeof(Pnew));
}

void EKF_Update(EKF_t* ekf, const EKF_Measurement_t* z) {
    if (!ekf->initialized || !z) return;

    /* 观测方程:
     * h(x) = [v_left_enc, v_right_enc, gyro_z_bias_corrected, accel_x_model]
     * v_left  = v_linear - omega * wheel_base/2
     * v_right = v_linear + omega * wheel_base/2
     * gyro_z  = omega + bias
     * accel_x = a_cmd (approx)
     */

    const float* x = ekf->x;
    const float half_base = ekf->wheel_base / 2.0f;

    /* 预测观测值 */
    float h[EKF_MEAS_DIM];
    h[0] = x[3] - x[4] * half_base;           /* 左轮速度 */
    h[1] = x[3] + x[4] * half_base;           /* 右轮速度 */
    h[2] = x[4] + x[5];                        /* gyro (含bias) */
    h[3] = 0;                                    /* 加速度 (简化) */

    /* 残差 y = z - h */
    float y[EKF_MEAS_DIM];
    y[0] = z->enc_left  - h[0];
    y[1] = z->enc_right - h[1];
    y[2] = z->gyro_z    - h[2];
    y[3] = z->accel_x   - h[3];

    /* 雅可比 H (4x6) */
    float H[EKF_MEAS_DIM][EKF_STATE_DIM];
    memset(H, 0, sizeof(H));
    H[0][3] = 1;  H[0][4] = -half_base;
    H[1][3] = 1;  H[1][4] =  half_base;
    H[2][4] = 1;  H[2][5] = 1;

    /* S = H*P*H' + R */
    float S[EKF_MEAS_DIM][EKF_MEAS_DIM];
    for (int i=0; i<EKF_MEAS_DIM; i++) {
        for (int j=0; j<EKF_MEAS_DIM; j++) {
            float s = 0;
            for (int k=0; k<EKF_STATE_DIM; k++)
                s += H[i][k] * ekf->P[k][j];  /* H*P */
            float sp = 0;
            for (int l=0; l<EKF_MEAS_DIM; l++)
                sp += s * H[j][l];              /* (H*P)*H' */
            S[i][j] = sp + ekf->R[i][j];
        }
    }

    /* 卡尔曼增益 K = P*H'*S^-1 (简化: 4x4求逆) */
    /* 这里用伴随矩阵法求逆 (对于小矩阵足够) */
    float K[EKF_STATE_DIM][EKF_MEAS_DIM];

    /* K = P * H' * inv(S) */
    float PHt[EKF_STATE_DIM][EKF_MEAS_DIM];
    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_MEAS_DIM; j++) {
            float s = 0;
            for (int k=0; k<EKF_STATE_DIM; k++)
                s += ekf->P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    }

    /* 4x4矩阵求逆 (高斯-约当消元法) */
    float S_inv[EKF_MEAS_DIM][EKF_MEAS_DIM];
    memcpy(S_inv, S, sizeof(S));

    int idx[EKF_MEAS_DIM];
    for (int i=0; i<EKF_MEAS_DIM; i++) idx[i] = i;

    for (int col=0; col<EKF_MEAS_DIM; col++) {
        /* 选主元 */
        int max_r = col;
        for (int r=col+1; r<EKF_MEAS_DIM; r++)
            if (fabs(S_inv[r][col]) > fabs(S_inv[max_r][col])) max_r = r;
        
        /* 交换行 */
        if (max_r != col) {
            for (int c=0; c<EKF_MEAS_DIM; c++) {
                float t = S_inv[col][c]; S_inv[col][c] = S_inv[max_r][c]; S_inv[max_r][c] = t;
            }
        }

        float pivot = S_inv[col][col];
        if (fabs(pivot) < 1e-10f) continue;

        for (int c=0; c<EKF_MEAS_DIM; c++)
            S_inv[col][c] /= pivot;
        S_inv[col][col] = 1.0f;

        for (int r=0; r<EKF_MEAS_DIM; r++) {
            if (r == col) continue;
            float factor = S_inv[r][col];
            for (int c=0; c<EKF_MEAS_DIM; c++)
                S_inv[r][c] -= factor * S_inv[col][c];
        }
    }

    /* K = PHt * S_inv */
    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_MEAS_DIM; j++) {
            float s = 0;
            for (int k=0; k<EKF_MEAS_DIM; k++)
                s += PHt[i][k] * S_inv[k][j];
            K[i][j] = s;
        }
    }

    /* 更新状态 x = x + K*y */
    for (int i=0; i<EKF_STATE_DIM; i++) {
        float correction = 0;
        for (int j=0; j<EKF_MEAS_DIM; j++)
            correction += K[i][j] * y[j];
        x[i] += correction;
    }

    /* 归一化角度 */
    x[2] = wrap_angle(x[2]);

    /* 更新协方差 P = (I-K*H)*P */
    float KH[EKF_STATE_DIM][EKF_STATE_DIM];
    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_STATE_DIM; j++) {
            float s = 0;
            for (int k=0; k<EKF_MEAS_DIM; k++)
                s += K[i][k] * H[k][j];
            KH[i][j] = s;
        }
    }

    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=0; j<EKF_STATE_DIM; j++) {
            float val = (i==j?1.0f:0.0f) - KH[i][j];
            float new_p = 0;
            for (int k=0; k<EKF_STATE_DIM; k++)
                new_p += val * ekf->P[k][j];
            ekf->P[i][j] = new_p;
        }
    }
}

void EKF_GetEstimate(const EKF_t* ekf, EKF_Estimate_t* est) {
    if (!ekf || !est) return;

    est->x = ekf->x[0];
    est->y = ekf->x[1];
    est->theta = ekf->x[2];
    est->v_linear = ekf->x[3];
    est->v_angular = ekf->x[4];
    est->vx = est->v_linear * cos(est->theta);
    est->vy = est->v_linear * sin(est->theta);

    /* 置信度基于trace(P) */
    float trace = 0;
    for (int i=0; i<3; i++) trace += ekf->P[i][i];
    est->confidence = 1.0f / (1.0f + trace);
}

void EKF_SetProcessNoise(EKF_t* ekf, float q_pos, float q_vel, float q_bias) {
    memset(ekf->Q, 0, sizeof(ekf->Q));
    ekf->Q[0][0] = q_pos;  ekf->Q[1][1] = q_pos;  ekf->Q[2][2] = q_pos*0.1f;
    ekf->Q[3][3] = q_vel;  ekf->Q[4][4] = q_vel;
    ekf->Q[5][5] = q_bias;
}

void EKF_SetMeasurementNoise(EKF_t* ekf, float r_enc, float r_gyro, float r_accel) {
    memset(ekf->R, 0, sizeof(ekf->R));
    ekf->R[0][0] = r_enc;   ekf->R[1][1] = r_enc;
    ekf->R[2][2] = r_gyro;
    ekf->R[3][3] = r_accel;
}

/**
 * @file    balance_v3_integration.c
 * @brief   STM32 底盘集成示例 - 将 V2协议/PID++/EKF 集成到现有 balance.c
 *
 * 集成步骤:
 * 1. 在 balance.c 头部添加 #include
 * 2. 替换 CAN_N_Usart_Control() 为 Protocol_ProcessByte()
 * 3. 替换 Incremental_PI_A/B 为 PID_Update()
 * 4. 添加 EKF_Update() 调用
 *
 * 使用方法:
 * - 将此文件内容复制到 R550_C30D/BALANCE/balance.c
 * - 或作为参考修改现有代码
 */

/* ==================== 第1步: 添加头文件 ==================== */
/*
// 在 balance.c 顶部添加:
#include "wheeltec_protocol_v2.h"
#include "pid_controller.h"
#include "ekf_estimator.h"
*/

/* ==================== 第2步: 全局变量声明 ==================== */
/*
// 在全局变量区域添加:

// 协议实例 (替代原来的 rxbuf[8])
ProtocolContext_t g_protocol;
ProtocolFrame_t g_tx_frame;

// PID++ 实例 (替代原来的 Incremental_PI_A/B/C/D)
PID_t g_pid_motor_a;   // 左轮
PID_t g_pid_motor_b;   // 右轮
PID_t g_pid_motor_c;   // 前轮(如有)
PID_t g_pid_motor_d;   // 后轮(如有)

// EKF 实例
EKF_t g_ekf;

// 状态数据 (用于响应状态请求)
StatusData_t g_status_data;

// 控制模式标志
uint8_t g_control_source = 0;  // 0=无, 1=遥控, 2=串口(V2协议), 3=APP

// 心跳定时器
uint32_t g_last_heartbeat_ms = 0;
bool g_heartbeat_ok = true;
*/

/* ==================== 第3步: 初始化函数 ==================== */
/*
// 在 System_Init() 或 Balance_task() 开始处添加:

void Control_Upgrade_Init(void) {
    // 初始化协议解析器
    Protocol_Init(&g_protocol);
    
    // 设置回调函数
    g_protocol.on_velocity_cmd = on_velocity_command_callback;
    g_protocol.on_pid_set = on_pid_set_callback;
    g_protocol.on_emergency_stop = on_emergency_stop_callback;
    g_protocol.on_status_request = on_status_request_callback;
    g_protocol.on_frame_error = on_frame_error_callback;
    
    // 初始化PID控制器 (使用优化参数)
    // 左轮PID
    PID_InitWithParams(&g_pid_motor_a, 300.0f, 50.0f, 20.0f);  // Kp,Ki,Kd
    g_pid_motor_a.Kf = 100.0f;           // 前馈增益
    g_pid_motor_a.integral_max = 5000.0f; // 积分限幅
    g_pid_motor_a.output_max = 16700.0f;  // 输出限幅 (PWM)
    g_pid_motor_a.deadzone = 0.02f;       // 死区 (m/s)
    g_pid_motor_a.rate_limit = 2000.0f;   // 变化率限制
    g_pid_motor_a.derivative_on_measurement = true;  // 微分作用于测量值
    
    // 右轮PID (相同参数，或根据实际情况调整)
    memcpy(&g_pid_motor_b, &g_pid_motor_a, sizeof(PID_t));
    
    // 初始化EKF
    EKF_Init(&g_ekf, 0.315f);  // wheel_base = 315mm
    
    printf("[UPGRADE] V2 Protocol + PID++ + EKF initialized\r\n");
}
*/

/* ==================== 第4步: 回调函数实现 ==================== */
/*
// 速度命令回调 (替代原来的 CAN_N_Usart_Control)
void on_velocity_command_callback(const VelocityCommand_t* cmd) {
    if (cmd == NULL) return;
    
    Move_X = cmd->vx;   // 前后速度 m/s
    Move_Y = cmd->vy;   // 左右速度 m/s (全向移动时使用)
    Move_Z = cmd->vz;   // 旋转角速度 rad/s
    
    g_control_source = 2;  // 标记为串口控制
    
    // 运动学解算 (保持原有逻辑)
    Kinematic_Analysis(Move_X, Move_Z);
}

// PID参数设置回调
void on_pid_set_callback(uint8_t motor_id, const PIDParams_t* params) {
    if (params == NULL || motor_id > 3) return;
    
    PID_t* target = NULL;
    switch(motor_id) {
        case 0: target = &g_pid_motor_a; break;
        case 1: target = &g_pid_motor_b; break;
        case 2: target = &g_pid_motor_c; break;
        case 3: target = &g_pid_motor_d; break;
    }
    
    if (target != NULL) {
        PID_SetFullParams(target,
            params->kp, params->ki, params->kd, params->kf,
            params->integral_max, params->output_max,
            params->deadzone, params->rate_limit);
        
        printf("[PID] Motor %d updated: P=%.1f I=%.1f D=%.1f F=%.1f\r\n",
               motor_id, params->kp, params->ki, params->kd, params->kf);
    }
}

// 急停回调
void on_emergency_stop_callback(void) {
    Move_X = 0; Move_Y = 0; Move_Z = 0;
    Kinematic_Analysis(0, 0);
    
    // 直接输出零PWM
    Set_Pwm(0, 0, 0, 0);
    
    g_control_source = 0;
    printf("[EMERGENCY] Stop!\r\n");
}

// 状态请求回调
void on_status_request_callback(void) {
    // 填充状态数据
    for (int i = 0; i < 4; i++) {
        g_status_data.motor_speed[i] = (i==0)?MOTOR_A.Encoder:(i==1)?MOTOR_B.Encoder:
                                          (i==2)?MOTOR_C.Encoder:MOTOR_D.Encoder;
        g_status_data.motor_pwm[i] = (i==0)?MOTOR_A.Motor_Pwm:(i==1)?MOTOR_B.Motor_Pwm:
                                        (i==2)?MOTOR_C.Motor_Pwm:MOTOR_D.Motor_Pwm;
        g_status_data.encoder_raw[i] = (i==0)?PI_A:(i==1)?PI_B:(i==2)?PI_C:PI_D;
    }
    
    // IMU数据 (如果有)
    g_status_data.euler[0] = Angle_Balance;  // roll
    g_status_data.euler[1] = Pitch;           // pitch  
    g_status_data.euler[2] = Gyro_Z;          // yaw rate
    
    // 电源数据
    g_status_data.battery_voltage = Voltage;
    g_status_data.fault_code = 0;  // TODO: 根据实际故障设置
    
    // 发送状态响应
    Protocol_SendStatus(&g_protocol, &g_status_data);
}

// 帧错误回调
void on_frame_error_callback(ErrorCode_t err) {
    static uint32_t err_count = 0;
    err_count++;
    if (err_count % 100 == 0) {  // 每100次错误打印一次
        printf("[PROTO] Error count: %lu (last: 0x%02X)\r\n", err_count, err);
    }
}
*/

/* ==================== 第5步: 修改主控任务 Balance_task() ==================== */
/*
// 替换原来的 Balance_task() 中的关键部分:

void Balance_task(void *pvParameters) {
    portTickType lastWakeTime = xTaskGetTickCount();
    
    // 初始化升级模块 (仅一次)
    static bool initialized = false;
    if (!initialized) {
        Control_Upgrade_Init();
        initialized = true;
    }
    
    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));  // 10ms
        
        // ===== 1. 读取编码器 (保持不变) =====
        Get_Velocity_Form_Encoder();
        
        // ===== 2. 处理串口接收 (V2协议替代原始方法) =====
        // 原来: if(rx_flag) { ... CAN_N_Usart_Control(); ... }
        // 改为:
        while (USART3_RX_available()) {
            uint8_t byte = USART3_ReceiveByte();
            
            // V2协议解析 (自动处理所有命令类型)
            bool frame_complete = Protocol_ProcessByte(&g_protocol, byte);
            
            if (frame_complete) {
                // 可选: LED闪烁表示收到有效帧
                LED_Toggle(LED_B);
            }
        }
        
        // ===== 3. 输入源选择 (保持原有逻辑) =====
        if(APP_ON_Flag) {
            Get_RC();
            g_control_source = 3;
        } else if(Remote_ON_Flag) {
            Remote_Control();
            g_control_source = 1;
        } else if(PS2_ON_Flag) {
            PS2_control();
            g_control_source = 3;
        } else {
            // 如果没有其他输入源，使用V2协议设定的值
            // Move_X/Y/Z 已在回调中设置
            Drive_Motor(Move_X, Move_Y, Move_Z);
        }
        
        // ===== 4. PID++控制 (替代原来的Incremental_PI) =====
        // 原来: MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
        // 改为:
        float dt = 0.01f;  // 100Hz -> 10ms
        
        MOTOR_A.Motor_Pwm = (int16_t)PID_Update(
            &g_pid_motor_a, 
            MOTOR_A.Target,     // setpoint (目标速度 m/s)
            MOTOR_A.Encoder,    // measurement (实际速度 m/s)
            dt                  // 时间间隔
        );
        
        MOTOR_B.Motor_Pwm = (int16_t)PID_Update(
            &g_pid_motor_b,
            MOTOR_B.Target,
            MOTOR_B.Encoder,
            dt
        );
        
        // C/D 轮同理...
        
        // ===== 5. EKF状态估计 (新增) =====
        EKF_Measurement_t ekf_z;
        ekf_z.enc_left = MOTOR_A.Encoder;
        ekf_z.enc_right = MOTOR_B.Encoder;
        ekf_z.gyro_z = Gyro_Z;       // 来自MPU6050
        ekf_z.accel_x = Accel_X;      // 来自MPU6050
        
        EKF_Control_t u = {.cmd_v = Move_X, .cmd_omega = Move_Z};
        EKF_Predict(&g_ekf, &u, dt);
        EKF_Update(&g_ekf, &ekf_z);
        
        // 可选: 发布EKF估计的位置 (通过串口或CAN)
        // EKF_Estimate_t est;
        // EKF_GetEstimate(&g_ekf, &est);
        // printf("EKF: x=%.2f y=%.2f th=%.2f\r\n", est.x, est.y, est.theta*RAD2DEG);
        
        // ===== 6. PWM限幅与输出 (保持不变) =====
        Limit_Pwm(16700);
        Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, 
                 MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm);
        
        // ===== 7. 心跳检测 (新增) =====
        uint32_t now_ms = HAL_GetTick();
        if (!Protocol_CheckHeartbeat(&g_protocol, now_ms)) {
            // 心跳丢失超过阈值，已自动触发急停
            LED_Set(LED_R);  // 红灯警告
        } else {
            LED_Set(LED_G);  // 绿灯正常
        }
    }
}
*/

/* ==================== 第6步: USART中断修改 ==================== */
/*
// 在 usart.c 的 USART3_IRQHandler 中:

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART3);
        
        // V2协议: 送入协议解析器 (非阻塞)
        // 注意: 这里只做简单缓存，实际解析在主循环中进行
        // 或者直接在ISR中调用 Protocol_ProcessByte() (如果性能允许)
        
        // 推荐方式: ISR中只存入环形缓冲区，主循环中取出处理
        RingBuffer_Put(&usart3_rx_ringbuf, data);
        
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
*/

/* ==================== 第7步: Flash参数表扩展 (可选) ==================== */
/*
// 在保存/读取Flash参数时，增加新字段:

typedef struct {
    // 原有参数
    float Velocity_KP;
    float Velocity_KI;
    // ...
    
    // 新增 V2 参数
    float Velocity_KD;          // 微分增益 (新增)
    float Velocity_KF;          // 前馈增益 (新增)
    float Integral_Max;         // 积分限幅 (新增)
    float Deadzone;             // 死区 (新增)
    float Rate_Limit;           // 变化率限制 (新增)
    
    // EKF 参数
    float EKF_Q_pos;            // 过程噪声-位置
    float EKF_Q_vel;            // 过程噪声-速度
    float EKF_Q_bias;           // 过程噪声-偏置
    float EKF_R_enc;            // 测量噪声-编码器
    float EKF_R_gyro;           // 测量噪声-陀螺仪
    
} FlashParams_V2_t;
*/

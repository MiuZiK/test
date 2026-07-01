/**
 * @file    wheeltec_protocol_v2.h
 * @brief   Wheeltec Protocol V2 - 可靠通信协议定义
 * @author  AI Assistant
 * @date    2026-04-27
 *
 * 协议特点:
 * - 帧头帧尾校验
 * - CRC16-CCITT 校验
 * - 序号防重复
 * - 心跳机制
 * - 命令应答
 */

#ifndef __WHEELTEC_PROTOCOL_V2_H
#define __WHEELTEC_PROTOCOL_V2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== 协议常量 ==================== */
#define PROTOCOL_HEAD             0xAA        /* 帧头 */
#define PROTOCOL_TAIL             0x55        /* 帧尾 */
#define PROTOCOL_VERSION          0x02        /* 版本号 V2 */
#define MAX_PAYLOAD_LEN           20         /* 最大载荷长度 */
#define FRAME_BUFFER_SIZE         64         /* 接收缓冲区大小 */

/* ==================== 命令码定义 ==================== */
typedef enum {
    /* --- 主机→从机 (ROS → STM32) --- */
    CMD_SET_VELOCITY       = 0x01,    /* 设置速度: Vx,Vy,Vz (float*3) */
    CMD_SET_PID            = 0x02,    /* 设置PID参数: KP,KI,KD (float*3) */
    CMD_REQUEST_STATUS     = 0x03,    /* 请求状态数据 */
    CMD_EMERGENCY_STOP     = 0x04,    /* 急停 */
    CMD_RESET_ENCODER      = 0x05,    /* 复位编码器 */
    CMD_SET_CAR_MODE       = 0x06,    /* 设置车型模式 */
    CMD_SET_PWM_DIRECT     = 0x07,    /* 直接设置PWM (调试用) */
    CMD_CALIBRATE_IMU      = 0x08,    /* IMU校准 */
    CMD_SET_LED            = 0x09,    /* 设置LED状态 */
    CMD_HEARTBEAT          = 0x0A,    /* 心跳包 */
    CMD_SET_PARAMS         = 0x0B,    /* 批量设置参数 */

    /* --- 从机→主机 (STM32 → ROS) --- */
    RESP_STATUS            = 0x81,    /* 状态响应 (电压/编码器/IMU) */
    RESP_ACK               = 0x82,    /* 应答确认 */
    RESP_NACK              = 0x83,    /* 应答失败 */
   _RESP_ERROR             = 0x84,    /* 错误报告 */
    RESP_HEARTBEAT         = 0x8A,    /* 心跳应答 */
    _RESP_DEBUG            = 0xFF     /* 调试信息 */

} CommandCode_t;

/* ==================== 车型模式 ==================== */
typedef enum {
    CAR_MODE_TANK          = 0x01,
    CAR_MODE_DIFF          = 0x02,
    CAR_MODE_MECANUM       = 0x03,
    CAR_MODE_ACKERMANN     = 0x04,
    CAR_MODE_OMNI          = 0x05,
    CAR_MODE_FOUR_WHEEL    = 0x06
} CarMode_t;

/* ==================== 错误码 ==================== */
typedef enum {
    ERR_NONE                = 0x00,
    ERR_CRC_FAIL            = 0x01,
    ERR_UNKNOWN_CMD         = 0x02,
    ERR_PAYLOAD_LEN         = 0x03,
    ERR_MOTOR_STALL         = 0x04,
    ERR_ENCODER_LOST        = 0x05,
    ERR_IMU_ERROR           = 0x06,
    ERR_LOW_VOLTAGE         = 0x07,
    ERR_OVER_CURRENT        = 0x08,
    ERR_OVER_TEMP           = 0x09,
    ERR_COMM_TIMEOUT        = 0x0A
} ErrorCode_t;

/* ==================== 数据结构 ==================== */

/** 帧结构 (发送/接收通用) */
typedef struct {
    uint8_t  head;                    /* 帧头 0xAA */
    uint8_t  cmd;                     /* 命令码 */
    uint8_t  len;                     /* 载荷长度 */
    uint8_t  seq;                     /* 序号 */
    uint8_t  payload[MAX_PAYLOAD_LEN];/* 载荷数据 */
    uint16_t crc;                     /* CRC16校验值 */
    uint8_t  tail;                    /* 帧尾 0x55 */
} ProtocolFrame_t;

/** 状态数据结构 (STM32 → ROS) */
typedef struct {
    /* 电机数据 */
    float    motor_speed[4];          /* 各轮速度 m/s */
    float    motor_pwm[4];            /* 各轮PWM */
    int32_t  encoder_raw[4];          /* 编码器原始值 */

    /* IMU数据 */
    float    accel[3];                /* 加速度 m/s^2 */
    float    gyro[3];                 /* 角速度 rad/s */
    float    euler[3];                /* 欧拉角 rad (roll,pitch,yaw) */
    float    quat[4];                 /* 四元数 */

    /* 电源数据 */
    float    battery_voltage;         /* 电池电压 V */
    float    battery_current;         /* 电池电流 A */
    uint8_t  battery_soc;             /* 电量百分比 % */

    /* 系统状态 */
    uint32_t uptime_ms;               /* 运行时间 ms */
    uint8_t  fault_code;              /* 故障码 */
    uint8_t  car_mode;                /* 当前车型模式 */
    uint8_t  control_mode;            /* 控制模式 (0=停止 1=遥控 2=串口 3=APP) */
    float    board_temp;              /* 板载温度 °C */

} StatusData_t;

/** 速度命令结构 (ROS → STM32) */
typedef struct {
    float vx;                         /* X方向速度 m/s */
    float vy;                         /* Y方向速度 m/s */
    float vz;                         /* Z轴角速度 rad/s */
} VelocityCommand_t;

/** PID参数结构 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float kf;                         /* 前馈增益 */
    float integral_max;               /* 积分限幅 */
    float output_limit;               /* 输出限幅 */
    float deadzone;                   /* 死区 */
} PIDParams_t;

/* ==================== 协议解析器状态 ==================== */
typedef enum {
    PARSE_STATE_IDLE,                 /* 空闲等待帧头 */
    PARSE_STATE_HEAD_FOUND,           /* 找到帧头，等待CMD */
    PARSE_STATE_CMD_FOUND,            /* 找到CMD，等待LEN */
    PARSE_STATE_LEN_FOUND,            /* 找到LEN，等待SEQ */
    PARSE_STATE_SEQ_FOUND,            /* 找到SEQ，接收载荷 */
    PARSE_STATE_PAYLOAD_RX,          /* 接收载荷中 */
    PARSE_STATE_CRC_LOW,             /* 接收CRC低字节 */
    PARSE_STATE_CRC_HIGH,            /* 接收CRC高字节 */
    PARSE_STATE_TAIL_FOUND,          /* 找到帧尾，完成 */
    PARSE_STATE_ERROR                 /* 错误状态 */
} ParseState_t;

/** 协议上下文 (实例化使用) */
typedef struct {
    /* 接收相关 */
    ParseState_t parse_state;         /* 解析状态 */
    ProtocolFrame_t rx_frame;         /* 接收帧缓冲 */
    uint8_t rx_payload_index;         /* 已接收载荷字节数 */
    uint16_t rx_crc_calc;             /* 计算中的CRC */

    /* 发送相关 */
    uint8_t tx_seq;                   /* 发送序号计数器 */
    uint8_t last_cmd;                 /* 最后发送的命令 */
    uint32_t last_tx_time;            /* 最后发送时间(ms) */
    uint8_t retry_count;              /* 重试次数 */

    /* 心跳相关 */
    uint32_t last_heartbeat_time;     /* 最后心跳时间(ms) */
    uint8_t heartbeat_miss_count;     /* 心跳丢失次数 */
    bool heartbeat_ok;                /* 心跳正常标志 */

    /* 回调函数 */
    void (*on_velocity_cmd)(const VelocityCommand_t* cmd);
    void (*on_pid_set)(uint8_t motor_id, const PIDParams_t* params);
    void (*on_emergency_stop)(void);
    void (*on_status_request)(void);
    void (*on_frame_error)(ErrorCode_t err);

} ProtocolContext_t;

/* ==================== API 函数声明 ==================== */

/**
 * @brief 初始化协议解析器
 * @param ctx 协议上下文指针
 */
void Protocol_Init(ProtocolContext_t* ctx);

/**
 * @brief 处理接收到的字节 (在中断或主循环中调用)
 * @param ctx 协议上下文
 * @param byte 接收到的字节
 * @return true=收到完整有效帧 false=继续接收
 */
bool Protocol_ProcessByte(ProtocolContext_t* ctx, uint8_t byte);

/**
 * @brief 发送速度命令
 * @param ctx 协议上下文
 * @param cmd 速度命令
 * @return 发送的字节数
 */
int Protocol_SendVelocity(ProtocolContext_t* ctx, const VelocityCommand_t* cmd);

/**
 * @brief 发送PID参数设置命令
 * @param ctx 协议上下文
 * @param motor_id 电机ID (0-3)
 * @param params PID参数
 */
int Protocol_SetPID(ProtocolContext_t* ctx, uint8_t motor_id, const PIDParams_t* params);

/**
 * @brief 发送急停命令
 * @param ctx 协议上下文
 */
int Protocol_EmergencyStop(ProtocolContext_t* ctx);

/**
 * @brief 发送心跳包
 * @param ctx 协议上下文
 */
int Protocol_SendHeartbeat(ProtocolContext_t* ctx);

/**
 * @brief 发送状态数据 (响应请求)
 * @param ctx 协议上下文
 * @param status 状态数据指针
 */
int Protocol_SendStatus(ProtocolContext_t* ctx, const StatusData_t* status);

/**
 * @brief 发送应答
 * @param ctx 协议上下文
 * @param cmd 对应的命令码
 * @param ack true=成功 NACK=失败
 */
int Protocol_SendAck(ProtocolContext_t* ctx, uint8_t cmd, bool ack);

/**
 * @brief 心跳检测 (在主循环中周期调用)
 * @param ctx 协议上下文
 * @param current_time_ms 当前时间(ms)
 * @return true=心跳正常 false=心跳超时
 */
bool Protocol_CheckHeartbeat(ProtocolContext_t* ctx, uint32_t current_time_ms);

/**
 * @brief CRC16-CCITT 计算
 * @param data 数据指针
 * @param length 长度
 * @return CRC16值
 */
uint16_t Protocol_CalcCRC16(const uint8_t* data, uint16_t length);

/**
 * @brief 组装发送帧
 * @param frame 帧结构体
 * @param cmd 命令码
 * @param payload 载荷数据
 * @param payload_len 载荷长度
 * @return 总帧长 (不含帧头前的可能填充)
 */
uint8_t Protocol_BuildFrame(
    ProtocolFrame_t* frame,
    uint8_t cmd,
    const uint8_t* payload,
    uint8_t payload_len
);

#ifdef __cplusplus
}
#endif

#endif /* __WHEELTEC_PROTOCOL_V2_H */

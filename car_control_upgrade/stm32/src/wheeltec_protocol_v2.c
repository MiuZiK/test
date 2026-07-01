/**
 * @file    wheeltec_protocol_v2.c
 * @brief   Wheeltec Protocol V2 - 可靠通信协议实现
 */

#include "wheeltec_protocol_v2.h"
#include <string.h>

/* ==================== CRC16-CCITT 查找表 ==================== */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x4395, 0x53B4, 0x63F7, 0x73D6,
    0x9349, 0x8368, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xE3FF, 0xF3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCFDD, 0xDFFC, 0xAF1B, 0xBF3A, 0x8F59, 0x9F78,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/* ==================== 内部函数 ==================== */

static void reset_parse_state(ProtocolContext_t* ctx) {
    ctx->parse_state = PARSE_STATE_IDLE;
    ctx->rx_payload_index = 0;
    ctx->rx_crc_calc = 0xFFFF;
}

/**
 * @brief 通过串口发送字节的回调 (需用户实现)
 */
extern void Protocol_SendByte(uint8_t byte);

/* ==================== API 实现 ==================== */

void Protocol_Init(ProtocolContext_t* ctx) {
    memset(ctx, 0, sizeof(ProtocolContext_t));
    ctx->parse_state = PARSE_STATE_IDLE;
    ctx->tx_seq = 0;
    ctx->heartbeat_ok = true;
    ctx->heartbeat_miss_count = 0;
}

uint16_t Protocol_CalcCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table[(crc >> 8) ^ data[i]];
    }
    return crc;
}

bool Protocol_ProcessByte(ProtocolContext_t* ctx, uint8_t byte) {
    switch (ctx->parse_state) {

    case PARSE_STATE_IDLE:
        if (byte == PROTOCOL_HEAD) {
            ctx->rx_frame.head = byte;
            ctx->rx_crc_calc = Protocol_CalcCRC16(&byte, 1);
            ctx->parse_state = PARSE_STATE_HEAD_FOUND;
        }
        break;

    case PARSE_STATE_HEAD_FOUND:
        ctx->rx_frame.cmd = byte;
        ctx->rx_crc_calc = Protocol_CalcCRC16(&byte, 1);
        /* 更新CRC: 需要重新计算从头开始，这里简化处理 */
        ctx->parse_state = PARSE_STATE_CMD_FOUND;
        break;

    case PARSE_STATE_CMD_FOUND:
        if (byte > MAX_PAYLOAD_LEN) {
            reset_parse_state(ctx);
            return false;
        }
        ctx->rx_frame.len = byte;
        ctx->parse_state = PARSE_STATE_LEN_FOUND;
        break;

    case PARSE_STATE_LEN_FOUND:
        ctx->rx_frame.seq = byte;
        ctx->rx_payload_index = 0;
        if (ctx->rx_frame.len == 0) {
            ctx->parse_state = PARSE_STATE_CRC_LOW;
        } else {
            ctx->parse_state = PARSE_STATE_SEQ_FOUND;
        }
        break;

    case PARSE_STATE_SEQ_FOUND:
    case PARSE_STATE_PAYLOAD_RX:
        if (ctx->rx_payload_index < ctx->rx_frame.len &&
            ctx->rx_payload_index < MAX_PAYLOAD_LEN) {
            ctx->rx_frame.payload[ctx->rx_payload_index++] = byte;
            if (ctx->rx_payload_index >= ctx->rx_frame.len) {
                ctx->parse_state = PARSE_STATE_CRC_LOW;
            }
        } else {
            reset_parse_state(ctx);
            return false;
        }
        break;

    case PARSE_STATE_CRC_LOW:
        ctx->rx_frame.crc = byte;  /* 低字节 */
        ctx->parse_state = PARSE_STATE_CRC_HIGH;
        break;

    case PARSE_STATE_CRC_HIGH:
        ctx->rx_frame.crc |= ((uint16_t)byte << 8);  /* 高字节 */
        ctx->parse_state = PARSE_STATE_TAIL_FOUND;
        break;

    case PARSE_STATE_TAIL_FOUND:
        if (byte != PROTOCOL_TAIL) {
            reset_parse_state(ctx);
            return false;
        }

        /* 帧接收完成，校验CRC */
        {
            uint16_t calc_crc = Protocol_CalcCRC16(
                &ctx->rx_frame.cmd,
                2 + 1 + 1 + ctx->rx_frame.len  /* cmd+len+seq+payload */
            );

            if (calc_crc != ctx->rx_frame.crc) {
                if (ctx->on_frame_error) {
                    ctx->on_frame_error(ERR_CRC_FAIL);
                }
                reset_parse_state(ctx);
                return false;
            }

            /* 帧有效！处理命令 */
            uint8_t cmd = ctx->rx_frame.cmd;

            switch (cmd) {
            case CMD_SET_VELOCITY:
                if (ctx->rx_frame.len >= 12 && ctx->on_velocity_cmd) {
                    VelocityCommand_t vel;
                    memcpy(&vel.vx,   &ctx->rx_frame.payload[0],  4);
                    memcpy(&vel.vy,   &ctx->rx_frame.payload[4],  4);
                    memcpy(&vel.vz,   &ctx->rx_frame.payload[8],  4);
                    ctx->on_velocity_cmd(&vel);
                }
                break;

            case CMD_SET_PID:
                if (ctx->rx_frame.len >= 28 && ctx->on_pid_set) {
                    PIDParams_t params;
                    uint8_t motor_id = ctx->rx_frame.payload[0];
                    memcpy(&params.kp,           &ctx->rx_frame.payload[1],  4);
                    memcpy(&params.ki,           &ctx->rx_frame.payload[5],  4);
                    memcpy(&params.kd,           &ctx->rx_frame.payload[9],  4);
                    memcpy(&params.kf,           &ctx->rx_frame.payload[13], 4);
                    memcpy(&params.integral_max, &ctx->rx_frame.payload[17], 4);
                    memcpy(&params.output_limit, &ctx->rx_frame.payload[21], 4);
                    memcpy(&params.deadzone,     &ctx->rx_frame.payload[25], 4);
                    ctx->on_pid_set(motor_id, &params);
                }
                break;

            case CMD_EMERGENCY_STOP:
                if (ctx->on_emergency_stop) {
                    ctx->on_emergency_stop();
                }
                break;

            case CMD_REQUEST_STATUS:
                if (ctx->on_status_request) {
                    ctx->on_status_request();
                }
                break;

            case CMD_HEARTBEAT:
                ctx->last_heartbeat_time = 0;  /* 由调用者设置时间戳 */
                ctx->heartbeat_miss_count = 0;
                ctx->heartbeat_ok = true;
                Protocol_SendAck(ctx, CMD_HEARTBEAT, true);
                break;

            default:
                if (ctx->on_frame_error) {
                    ctx->on_frame_error(ERR_UNKNOWN_CMD);
                }
                break;
            }

            /* 发送应答 (非心跳命令) */
            if (cmd != CMD_HEARTBEAT && cmd != CMD_REQUEST_STATUS) {
                Protocol_SendAck(ctx, cmd, true);
            }

            reset_parse_state(ctx);
            return true;  /* 收到有效帧 */
        }

    default:
        reset_parse_state(ctx);
        break;
    }

    return false;
}

uint8_t Protocol_BuildFrame(
    ProtocolFrame_t* frame,
    uint8_t cmd,
    const uint8_t* payload,
    uint8_t payload_len
) {
    if (payload_len > MAX_PAYLOAD_LEN) {
        payload_len = MAX_PAYLOAD_LEN;
    }

    frame->head = PROTOCOL_HEAD;
    frame->cmd  = cmd;
    frame->len  = payload_len;
    frame->seq  = 0;  /* 由外部设置 */
    frame->tail = PROTOCOL_TAIL;

    if (payload != NULL && payload_len > 0) {
        memcpy(frame->payload, payload, payload_len);
    } else {
        memset(frame->payload, 0, payload_len);
    }

    /* 计算CRC: 从cmd到payload结束 */
    frame->crc = Protocol_CalcCRC16(
        (const uint8_t*)&frame->cmd,
        2 + 1 + 1 + payload_len
    );

    return 1 + 1 + 1 + 1 + payload_len + 2 + 1;  /* 总帧长 */
}

static void send_raw_frame(ProtocolContext_t* ctx, const ProtocolFrame_t* frame) {
    /* 发送帧头 */
    Protocol_SendByte(frame->head);

    /* 发送CMD, LEN, SEQ */
    Protocol_SendByte(frame->cmd);
    Protocol_SendByte(frame->len);
    Protocol_SendByte(frame->seq);

    /* 发送载荷 */
    for (uint8_t i = 0; i < frame->len; i++) {
        Protocol_SendByte(frame->payload[i]);
    }

    /* 发送CRC (低字节在前) */
    Protocol_SendByte(frame->crc & 0xFF);
    Protocol_SendByte((frame->crc >> 8) & 0xFF);

    /* 发送帧尾 */
    Protocol_SendByte(frame->tail);
}

int Protocol_SendVelocity(ProtocolContext_t* ctx, const VelocityCommand_t* cmd) {
    ProtocolFrame_t frame;
    uint8_t payload[12];

    memcpy(&payload[0], &cmd->vx, 4);
    memcpy(&payload[4], &cmd->vy, 4);
    memcpy(&payload[8], &cmd->vz, 4);

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, CMD_SET_VELOCITY, payload, 12);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    ctx->last_tx_time = 0;  /* 由调用者设置 */
    ctx->last_cmd = CMD_SET_VELOCITY;

    return 17;  /* 1+1+1+1+12+2+1 */
}

int Protocol_SetPID(ProtocolContext_t* ctx, uint8_t motor_id, const PIDParams_t* params) {
    ProtocolFrame_t frame;
    uint8_t payload[29];  /* 1(id) + 28(params) */

    payload[0] = motor_id;
    memcpy(&payload[1],  &params->kp,           4);
    memcpy(&payload[5],  &params->ki,           4);
    memcpy(&payload[9],  &params->kd,           4);
    memcpy(&payload[13], &params->kf,           4);
    memcpy(&payload[17], &params->integral_max, 4);
    memcpy(&payload[21], &params->output_limit, 4);
    memcpy(&payload[25], &params->deadzone,     4);

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, CMD_SET_PID, payload, 29);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    return 34;
}

int Protocol_EmergencyStop(ProtocolContext_t* ctx) {
    ProtocolFrame_t frame;

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, CMD_EMERGENCY_STOP, NULL, 0);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    return 6;
}

int Protocol_SendHeartbeat(ProtocolContext_t* ctx) {
    ProtocolFrame_t frame;

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, CMD_HEARTBEAT, NULL, 0);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    ctx->last_heartbeat_time = 0;  /* 由调用者设置 */

    return 6;
}

int Protocol_SendStatus(ProtocolContext_t* ctx, const StatusData_t* status) {
    ProtocolFrame_t frame;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t offset = 0;

    /* 电机速度 (4*float = 16 bytes) */
    memcpy(&payload[offset], status->motor_speed, 16); offset += 16;

    /* 编码器原始值 (4*int32 = 16 bytes) - 如果空间不够可以省略 */
    if (offset + 16 <= MAX_PAYLOAD_LEN) {
        memcpy(&payload[offset], status->encoder_raw, 16); offset += 16;
    }

    /* IMU数据 (euler 3*float = 12 bytes) */
    if (offset + 12 <= MAX_PAYLOAD_LEN) {
        memcpy(&payload[offset], status->euler, 12); offset += 12;
    }

    /* 电池电压 (1*float = 4 bytes) */
    if (offset + 4 <= MAX_PAYLOAD_LEN) {
        memcpy(&payload[offset], &status->battery_voltage, 4); offset += 4;
    }

    /* 故障码 (1 byte) */
    if (offset + 1 <= MAX_PAYLOAD_LEN) {
        payload[offset++] = status->fault_code;
    }

    /* 车型模式 (1 byte) */
    if (offset + 1 <= MAX_PAYLOAD_LEN) {
        payload[offset++] = status->car_mode;
    }

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, RESP_STATUS, payload, offset);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    return 6 + offset;
}

int Protocol_SendAck(ProtocolContext_t* ctx, uint8_t cmd, bool ack) {
    ProtocolFrame_t frame;
    uint8_t payload[2];

    payload[0] = cmd;
    payload[1] = ack ? RESP_ACK : RESP_NACK;

    ctx->tx_seq++;
    Protocol_BuildFrame(&frame, payload[1], payload, 2);
    frame.seq = ctx->tx_seq;

    send_raw_frame(ctx, &frame);
    return 8;
}

bool Protocol_CheckHeartbeat(ProtocolContext_t* ctx, uint32_t current_time_ms) {
    #define HEARTBEAT_TIMEOUT_MS  500   /* 心跳超时 500ms */
    #define HEARTBEAT_MAX_MISS    3      /* 最大丢失次数 */

    if (ctx->last_heartbeat_time > 0 &&
        (current_time_ms - ctx->last_heartbeat_time) > HEARTBEAT_TIMEOUT_MS) {

        ctx->heartbeat_miss_count++;

        if (ctx->heartbeat_miss_count >= HEARTBEAT_MAX_MISS) {
            ctx->heartbeat_ok = false;
            /* 触发急停 */
            if (ctx->on_emergency_stop) {
                ctx->on_emergency_stop();
            }
            return false;
        }
    }

    return ctx->heartbeat_ok;
}

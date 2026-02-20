/*
 * zdtEmm.c
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */
#include "zdtEmm.h"
#include "zdtCan.h"

ZDT_Motor_t motor1;

void ZDT_Emm_Init(ZDT_Motor_t *motor, uint8_t id) {
    motor->node_id = id;
    motor->target_speed = 0;
    motor->actual_speed = 0;
    motor->acc = 0; // 0=直接启动，建议测试用0
    motor->dir = 0;
    motor->enabled = 0;
}
// 【新增】发送使能命令 (0xF3)
void ZDT_Emm_Enable(ZDT_Motor_t *motor) {
    uint8_t tx_data[6];
    tx_data[0] = 0xF3;  // 使能命令
    tx_data[1] = 0xAB;
    tx_data[2] = 0x01;  // 使能
    tx_data[3] = 0x00;  // 不同步
    tx_data[4] = 0x6B;  // 校验
    tx_data[5] = 0x00;

    ZDT_CAN_Send_ExtId(motor->node_id << 8, tx_data, 5);
    motor->enabled = 1;
}
// 发送速度命令 (0xF6)
void ZDT_Emm_SetSpeed(ZDT_Motor_t *motor, float speed_rpm) {
	// 如果没使能，先使能
	    if (!motor->enabled) {
	        ZDT_Emm_Enable(motor);
	        return;  // 本次只发送使能，下次再发速度
	    }
	uint8_t tx_data[8];
    uint16_t vel_int;

    motor->target_speed = speed_rpm;

    if (speed_rpm < 0) {
        motor->dir = 1;
        speed_rpm = -speed_rpm;
    } else {
        motor->dir = 0;
    }

    vel_int = (uint16_t)speed_rpm;

    tx_data[0] = 0xF6; // 功能码
    tx_data[1] = motor->dir;
    tx_data[2] = (vel_int >> 8) & 0xFF;
    tx_data[3] = vel_int & 0xFF;
    tx_data[4] = motor->acc;
    tx_data[5] = 0x00; // 同步
    tx_data[6] = 0x6B; // 校验
    tx_data[7] = 0x00;

    // ZDT V5 协议 ID 格式: NodeID << 8
    ZDT_CAN_Send_ExtId(motor->node_id << 8, tx_data, 7);
}
void ZDT_Emm_ReadSpeed(ZDT_Motor_t *motor) {
    uint8_t tx_data[2];
    tx_data[0] = 0x35; // 功能码：读取电机实时转速
    tx_data[1] = 0x6B; // 校验

    // 发送 2 字节 payload
    ZDT_CAN_Send_ExtId(motor->node_id << 8, tx_data, 2);
}
// 解析电机反馈
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len) {
    // ✅ 尝试两种 ID 解析方式（调试用）
    uint8_t sender_id_high = (ExtId >> 8) & 0xFF;
    uint8_t sender_id_low = ExtId & 0xFF;

    // 优先匹配高字节（根据说明书）
    if (sender_id_high != motor1.node_id) {
        // 如果高字节不匹配，尝试低字节（某些固件版本可能不同）
        if (sender_id_low != motor1.node_id) {
            return;  // ID 不匹配，丢弃
        }
    }

    if (Len < 3) return;

    uint8_t cmd = Data[0];

    // ✅ 根据功能码区分处理
    if (cmd == 0x35) {
        // --- 处理读取速度响应 (0x35) ---
        // 说明书格式：地址 + 0x35 + 符号 + 速度 (2 字节) + 校验
        if (Len >= 5) {
            uint8_t sign = Data[1];
            uint16_t speed_uint = (Data[2] << 8) | Data[3];
            float real_speed = (float)speed_uint;

            if (sign == 0x01) {
                real_speed = -real_speed;
            }

            motor1.actual_speed = real_speed;  // ✅ 更新速度
        }
    }
    else if (cmd == 0xF6) {
        // --- 处理速度控制应答 (0xF6) ---
        // Data[1] = 命令状态 (0x02=成功，0xE2=失败)
        if (Data[1] == 0xE2) {
            // 失败，可能是没使能
            motor1.enabled = 0;
        }
    }
    else if (cmd == 0xF3) {
        // --- 处理使能应答 (0xF3) ---
        if (Data[1] == 0x02) {
            motor1.enabled = 1;  // ✅ 使能成功
        }
    }
}

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
}

// 发送速度命令 (0xF6)
void ZDT_Emm_SetSpeed(ZDT_Motor_t *motor, float speed_rpm) {
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

// 解析电机反馈
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len) {
	uint8_t sender_id = ExtId & 0xFF;

	// 如果 ID 不匹配，直接退出
	    if (sender_id != motor1.node_id) return;

	    // 2. 解析数据 (基于 ZDT Emm V5 通用协议)
	    // 无论是 0xF6(应答) 还是 0x36(自动上报)，速度数据通常都在 Data[2] 和 Data[3]
	    // Data[0]: 功能码
	    // Data[1]: 方向 (0:CW, 1:CCW)
	    // Data[2]: 速度高 8 位
	    // Data[3]: 速度低 8 位

	    // 提取方向
	    uint8_t dir = Data[1];

	    // 提取速度 (uint16_t)
	    uint16_t speed_uint = (Data[2] << 8) | Data[3];

	    // 转换为 float
	    float real_speed = (float)speed_uint;

	    // 3. 根据方向处理正负号
	    // 注意：这里的方向定义(0/1)要和 SetSpeed 里的保持一致
	    if (dir == 1) {
	        real_speed = -real_speed;
	    } else {
	        real_speed = real_speed;
	    }

	    // 4. 更新全局变量
	    motor1.actual_speed = real_speed;

	    // (可选) 如果还需要位置信息，通常在 Data[4]~[7] 或其他功能码中，具体看手册
	}

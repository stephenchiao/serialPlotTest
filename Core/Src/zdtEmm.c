/*
 * zdtEmm.c
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */
#include "zdtEmm.h"
#include "zdtCan.h"

ZDT_Motor_t motors[4];

void ZDT_Emm_InitAll(void) {
    // 初始化 4 个电机 ID: 1, 2, 3, 4
	 motors[0].node_id = 1;  // ID 1 左后
	    motors[0].target_speed = 0;
	    motors[0].actual_speed = 0;

	    motors[1].node_id = 2;  // ID 2 左前
	    motors[1].target_speed = 0;
	    motors[1].actual_speed = 0;

	    motors[2].node_id = 3;  // ID 3 右前
	    motors[2].target_speed = 0;
	    motors[2].actual_speed = 0;

	    motors[3].node_id = 4;  // ID 4 右后
	    motors[3].target_speed = 0;
	    motors[3].actual_speed = 0;
}
void ZDT_Emm_SetSpeedByID(uint8_t id, float speed_rpm) {
    uint8_t tx_data[7];
    uint16_t vel_int;
    uint8_t dir = (speed_rpm < 0) ? 1 : 0;
    vel_int = (uint16_t)(speed_rpm < 0 ? -speed_rpm : speed_rpm);

    // 依据：P51 5.3.7 速度模式控制（Emm）
    tx_data[0] = 0xF6;                    // 功能码
    tx_data[1] = dir;                     // 方向
    tx_data[2] = (vel_int >> 8) & 0xFF;   // 速度高字节
    tx_data[3] = vel_int & 0xFF;          // 速度低字节
    tx_data[4] = 0;                       // 加速度
    tx_data[5] = 0x00;                    // 同步标志
    tx_data[6] = 0x6B;                    // 校验码，依据：P106 8.1

    // 依据：P40 4.2.1 CAN 扩展帧 ID = (Addr << 8) | Packet
    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 7);
}
void ZDT_Emm_ReadSpeedByID(uint8_t id) {
    uint8_t tx_data[2];
    tx_data[0] = 0x35;  // 功能码，依据：P67 5.5.11
    tx_data[1] = 0x6B;  // 校验码
    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 2);
}
// 解析电机反馈
// ✅ 修改：接收回调支持所有电机
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len) {
    uint8_t sender_id = (ExtId >> 8) & 0xFF;
    if (Len < 3) return;

    // ✅ 修复：功能码在 Data[0]，依据：P41 4.2.2
    uint8_t func_code = Data[0];

    // 查找对应的电机结构体
    for(int i=0; i<4; i++) {
        if (motors[i].node_id == sender_id) {
            if (func_code == 0x35 && Len >= 5) {  // 读取转速回复，依据：P67 5.5.11
                uint8_t sign = Data[1];
                uint16_t speed_uint = (Data[2] << 8) | Data[3];
                float real_speed = (float)speed_uint;
                if (sign == 0x01) real_speed = -real_speed;
                motors[i].actual_speed = real_speed; // 更新实际速度用于里程计
            }
            break;
        }
    }
}
void ZDT_Emm_EnableByID(uint8_t id) {
    uint8_t tx_data[4];
    // 依据：P48 5.3.2 电机使能控制 (假设协议：0xF3 + 使能位 + 状态位 + 校验)
    // 注意：请根据你手里的 ZDT 驱动器手册核对具体字节含义
    tx_data[0] = 0xF3; // 功能码
    tx_data[1] = 0xAB; // 使能
    tx_data[2] = 0x00; // 0x00为使能，0x01为释放
    tx_data[3] = 0x6B; // 校验码

    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 4);
}


//调试用
void ZDT_Emm_SetSingleMotorSpeed(uint8_t id, float speed_rpm) {
    uint8_t tx_data[7];
    uint16_t vel_int;
    uint8_t dir = (speed_rpm < 0) ? 1 : 0;
    vel_int = (uint16_t)(speed_rpm < 0 ? -speed_rpm : speed_rpm);

    tx_data[0] = 0xF6;                    // 功能码
    tx_data[1] = dir;                     // 方向 00/01
    tx_data[2] = (vel_int >> 8) & 0xFF;   // 速度高字节
    tx_data[3] = vel_int & 0xFF;          // 速度低字节
    tx_data[4] = 0;                       // 加速度 00-FF
    tx_data[5] = 0x00;                    // 同步标志 00=立即执行
    tx_data[6] = 0x6B;                    // 校验码，依据：P106 8.1

    // 依据：P40 4.2.1 CAN 扩展帧 ID = (Addr << 8) | Packet
    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 7);
}

// ✅ 新增：读取单个电机速度
// 依据：P67 5.5.11 读取电机实时转速
void ZDT_Emm_ReadSingleMotorSpeed(uint8_t id) {
    uint8_t tx_data[2];
    tx_data[0] = 0x35;  // 功能码
    tx_data[1] = 0x6B;  // 校验码
    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 2);
}

// ✅ 新增：使能/失能单个电机
// 依据：P48 5.3.2 电机使能控制
void ZDT_Emm_EnableSingleMotor(uint8_t id, uint8_t enable) {
    uint8_t tx_data[5];
    tx_data[0] = 0xF3;  // 功能码
    tx_data[1] = 0xAB;  // 辅助码
    tx_data[2] = enable ? 0x01 : 0x00;  // 使能状态 01=使能，00=失能
    tx_data[3] = 0x00;  // 同步标志 00=立即执行
    tx_data[4] = 0x6B;  // 校验码
    ZDT_CAN_Send_ExtId((id << 8) | 0, tx_data, 5);
}

// ✅ 新增：获取单个电机实际速度（从 motors 数组读取）
float ZDT_Emm_GetSingleMotorSpeed(uint8_t id) {
    for(int i=0; i<4; i++) {
        if (motors[i].node_id == id) {
            return motors[i].actual_speed;
        }
    }
    return 0.0f;
}

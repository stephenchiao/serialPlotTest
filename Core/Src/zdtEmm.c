#include "zdtEmm.h"
#include "zdtCan.h"

ZDT_Motor_t motor1;

void ZDT_Emm_Init(ZDT_Motor_t *motor, uint8_t id) {
    motor->node_id = id;
    motor->target_speed = 0;
    motor->actual_speed = 0;
    motor->acc = 0;
    motor->dir = 0;
}

// 仅下发速度指令
void ZDT_Emm_SetSpeed(ZDT_Motor_t *motor, float speed_rpm) {
    uint8_t tx_data[8];
    uint16_t vel_int;

    if (speed_rpm < 0) {
        motor->dir = 1;
        speed_rpm = -speed_rpm;
    } else {
        motor->dir = 0;
    }

    vel_int = (uint16_t)speed_rpm;
    tx_data[0] = 0xF6;  // 功能码
    tx_data[1] = motor->dir;
    tx_data[2] = (vel_int >> 8) & 0xFF;
    tx_data[3] = vel_int & 0xFF;
    tx_data[4] = motor->acc;
    tx_data[5] = 0x00;
    tx_data[6] = 0x6B;
    tx_data[7] = 0x00;

    uint32_t can_id = (motor->node_id << 8) | 0;
    ZDT_CAN_Send_ExtId(can_id, tx_data, 8);
}

// 空实现（为了链接不报错）
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len) {
    // 先不接收反馈
}


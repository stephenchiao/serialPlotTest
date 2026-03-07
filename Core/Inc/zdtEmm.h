/*
 * zdtEmm.h
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */

#ifndef INC_ZDTEMM_H_
#define INC_ZDTEMM_H_

#include <stdint.h>

typedef struct {
    uint8_t node_id;
    float target_speed;
    float actual_speed;
    uint8_t dir;
    uint8_t acc;
    uint8_t enabled;
} ZDT_Motor_t;


extern ZDT_Motor_t motors[4];

#define MOTOR_ID_BL  1  // Back-Left  左后
#define MOTOR_ID_FL  2  // Front-Left 左前
#define MOTOR_ID_FR  3  // Front-Right 右前
#define MOTOR_ID_BR  4  // Back-Right 右后

void ZDT_Emm_InitAll(void);
void ZDT_Emm_SetSpeedByID(uint8_t id, float speed_rpm);
void ZDT_Emm_ReadSpeedByID(uint8_t id);
void ZDT_Emm_EnableByID(uint8_t id);
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len);

void ZDT_Emm_ReadPositionByID(uint8_t id);//读取单个电机位置

int32_t ZDT_Emm_GetSingleMotorPosition(uint8_t id);//获取单个电机位置
//调试用
void ZDT_Emm_SetSingleMotorSpeed(uint8_t id, float speed_rpm);      // 设置单个电机速度
void ZDT_Emm_ReadSingleMotorSpeed(uint8_t id);                       // 读取单个电机速度
void ZDT_Emm_EnableSingleMotor(uint8_t id, uint8_t enable);          // 使能/失能单个电机
float ZDT_Emm_GetSingleMotorSpeed(uint8_t id);                       // 获取单个电机实际速度
#endif

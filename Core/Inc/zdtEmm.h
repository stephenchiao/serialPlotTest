/*
 * zdtEmm.h
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */

#ifndef INC_ZDTEMM_H_
#define INC_ZDTEMM_H_

#include <stdint.h>
#include "motor_monitor.h"

typedef struct {
    uint8_t node_id;
    float target_speed;
    float actual_speed;
    uint8_t dir;
    uint8_t acc;
    uint8_t enabled;
} ZDT_Motor_t;

typedef enum {
    ZDT_PROTOCOL_EMM = 0,
    ZDT_PROTOCOL_X = 1
} ZDT_Protocol_t;

typedef struct {
    uint8_t motor_id;
    uint8_t function_code;
    uint8_t value;
    float speed_rpm;
} ZDT_MotorEvent_t;


extern ZDT_Motor_t motors[4];

#define MOTOR_ID_BL  1  // Back-Left  左后
#define MOTOR_ID_FL  2  // Front-Left 左前
#define MOTOR_ID_FR  3  // Front-Right 右前
#define MOTOR_ID_BR  4  // Back-Right 右后

/*
 * 对外接口统一收敛为 ZDT_Emm_*ByID 一套命名（id 取 1..4）。
 * 原先并存的 *SingleMotor* 别名已删除，避免同一功能两套入口。
 */
void ZDT_Emm_InitAll(void);
void ZDT_Emm_RefreshEnables(void);
void ZDT_Emm_GetFeedback(MotorFeedback output[4]);
/* 停车：motor_mask 位掩码，bit0..3 对应 ID1..4。 */
uint8_t ZDT_Emm_StopMask(uint8_t motor_mask);
/* 每次下发非零速度都会递增，用于让上层识别运动代次是否变化。 */
uint32_t ZDT_Emm_MotionGeneration(void);
uint8_t ZDT_Emm_SetSpeedByID(uint8_t id, float speed_rpm);
uint8_t ZDT_Emm_ReadSpeedByID(uint8_t id);
uint8_t ZDT_Emm_ReadStatusByID(uint8_t id);
uint8_t ZDT_Emm_EnableByID(uint8_t id, uint8_t enable);
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len);
void ZDT_Emm_SetProtocol(ZDT_Protocol_t protocol);
ZDT_Protocol_t ZDT_Emm_GetProtocol(void);
uint8_t ZDT_Emm_PollEvent(ZDT_MotorEvent_t *event);

#endif

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

extern ZDT_Motor_t motor1;

void ZDT_Emm_Init(ZDT_Motor_t *motor, uint8_t id);
void ZDT_Emm_Enable(ZDT_Motor_t *motor);  // ✅ 新增
void ZDT_Emm_SetSpeed(ZDT_Motor_t *motor, float speed_rpm);
void ZDT_Emm_ReadSpeed(ZDT_Motor_t *motor);
void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len);

#endif

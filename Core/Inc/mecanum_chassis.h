/*
 * mecanum_chassis.h
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */

#ifndef INC_MECANUM_CHASSIS_H_
#define INC_MECANUM_CHASSIS_H_

#include "main.h"

/* 机器人底盘物理参数定义 */
#define WHEEL_DIAMETER  0.075f      // 轮子直径 75mm (单位:m)
#define ROBOT_W         0.229f      // 轮距 (左右轮距离) (单位:m)
#define ROBOT_H         0.255f      // 轴距 (前后轮距离) (单位:m)

/* 外部调用函数声明 */
void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br);
float MsToRpm(float v_ms);
void SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br);
void ReadAllMotorsSpeed(void);
void StopAllMotors(void);



#endif /* INC_MECANUM_CHASSIS_H_ */

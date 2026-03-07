/*
 * mecanum_chassis.c
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */
/*
 * @brief  麦克纳姆轮运动学逆解算 (坐标系与 OPS-9 对齐)
 * @param  Vx: 沿 X 轴的速度 (正值向右平移，单位 m/s)
 * @param  Vy: 沿 Y 轴的速度 (正值向前平移，单位 m/s)
 * @param  Vz: 绕 Z 轴的角速度 (正值逆时针旋转，单位 rad/s)
 * @retval 算出各轮目标线速度，存入指针
 */
#include "mecanum_chassis.h"
#include "zdtEmm.h"

void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br) {
    float L = (ROBOT_H / 2.0f) + (ROBOT_W / 2.0f);

    *V_bl = -Vx + Vy - Vz * L;  // ID 1 左后
    *V_fl =  Vx + Vy - Vz * L;  // ID 2 左前
    *V_fr =  Vx - Vy - Vz * L;  // ID 3 右前
    *V_br = -Vx - Vy - Vz * L;  // ID 4 右后
}

/*
 * @brief  线速度 (m/s) 转 电机转速 (RPM)
 */
float MsToRpm(float v_ms) {
    if (WHEEL_DIAMETER <= 0.0f) return 0.0f;
    // V = RPM * π * D / 60  =>  RPM = V * 60 / (π * D)
    return v_ms * 60.0f / (3.1415926f * WHEEL_DIAMETER);
}

/*
 * @brief  设置 4 个轮子速度并下发至 CAN 节点
 */
void SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br) {
    ZDT_Emm_SetSpeedByID(1, MsToRpm(V_bl));  // ID 1: 左后
    ZDT_Emm_SetSpeedByID(2, MsToRpm(V_fl));  // ID 2: 左前
    ZDT_Emm_SetSpeedByID(3, MsToRpm(V_fr));  // ID 3: 右前
    ZDT_Emm_SetSpeedByID(4, MsToRpm(V_br));  // ID 4: 右后
}

/*
 * @brief  向 4 个电机发送读取速度的指令
 */
void ReadAllMotorsSpeed(void) {
    ZDT_Emm_ReadSpeedByID(1);
    ZDT_Emm_ReadSpeedByID(2);
    ZDT_Emm_ReadSpeedByID(3);
    ZDT_Emm_ReadSpeedByID(4);
}

/*
 * @brief  紧急停止所有电机
 */
void StopAllMotors(void) {
    SetAllMotorsSpeed(0.0f, 0.0f, 0.0f, 0.0f);
}




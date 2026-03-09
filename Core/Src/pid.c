/*
 * pid.c
 *
 *  Created on: Mar 9, 2026
 *      Author: steph
 */
#include "pid.h"

/**
 * @brief  初始化 PID 控制器
 * @param  pid          PID 结构体指针
 * @param  Kp           比例系数
 * @param  Ki           积分系数
 * @param  Kd           微分系数
 * @param  max_out      最大输出限幅 (例如电机的最大 RPM)
 * @param  max_integral 最大积分限幅 (防止长时间卡住导致积分爆表)
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float max_out, float max_integral)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->max_out = max_out;
    pid->max_integral = max_integral;
}

/**
 * @brief  设置 PID 的目标值
 */
void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  重置 PID 状态 (清空历史误差和积分)
 * @note   通常在改变目标、重新发车时调用
 */
void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}

/**
 * @brief  执行一次位置式 PID 计算
 * @param  pid          PID 结构体指针
 * @param  current_val  传感器当前真实值 (如 OPS-9 的当前 X 坐标)
 * @return float        PID 计算输出的控制量 (如电机的目标速度)
 */
float PID_Calc(PID_Controller *pid, float current_val)
{
    // 1. 计算当前偏差 (目标值 - 当前值)
    pid->error = pid->target - current_val;

    // 2. 累加积分 (只有当需要积分时才算，防止浮点数越界)
    if (pid->Ki != 0.0f)
    {
        pid->integral += pid->error;

        // 积分抗饱和处理 (Anti-windup)
        if (pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if (pid->integral < -pid->max_integral) {
            pid->integral = -pid->max_integral;
        }
    }

    // 3. 计算 P, I, D 三项各自的输出
    float p_out = pid->Kp * pid->error;
    float i_out = pid->Ki * pid->integral;
    float d_out = pid->Kd * (pid->error - pid->last_error);

    // 4. 将三项相加，得到总输出
    float total_out = p_out + i_out + d_out;

    // 5. 整体输出限幅 (限制小车的最高速度)
    if (total_out > pid->max_out) {
        total_out = pid->max_out;
    } else if (total_out < -pid->max_out) {
        total_out = -pid->max_out;
    }

    // 6. 记录本次偏差，作为下一次求微分的依据
    pid->last_error = pid->error;

    // 7. 返回最终需要输出给执行机构的数值
    return total_out;
}




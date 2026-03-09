/*
 * pid.h
 *
 *  Created on: Mar 9, 2026
 *      Author: steph
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// PID 控制器结构体定义
typedef struct {
    // === 调节参数 ===
    float Kp;           // 比例系数 (Proportional)
    float Ki;           // 积分系数 (Integral)
    float Kd;           // 微分系数 (Derivative)

    // === 运行状态 ===
    float target;       // 目标设定值 (Target/Setpoint)
    float error;        // 当前偏差 (Error)
    float last_error;   // 上次偏差 (Last Error)
    float integral;     // 误差积分累加值 (Integral term)

    // === 限幅保护 ===
    float max_out;      // 输出限幅 (防止电机满载暴走)
    float max_integral; // 积分限幅 (防止积分饱和导致过冲)
} PID_Controller;

// 初始化 PID 控制器参数
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float max_out, float max_integral);

// 设置目标值
void PID_SetTarget(PID_Controller *pid, float target);

// 清空 PID 历史状态 (用于小车到达目标后，准备下一次移动前清空积分)
void PID_Reset(PID_Controller *pid);

// PID 核心计算函数 (放在定时器中断或主循环中周期性调用)
float PID_Calc(PID_Controller *pid, float current_val);



#endif /* INC_PID_H_ */

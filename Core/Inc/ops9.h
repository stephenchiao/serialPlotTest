/*
 * ops9.h
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */

#ifndef INC_OPS9_H_
#define INC_OPS9_H_

#include "main.h"

// 外部可读取的机器人绝对坐标和角度
extern float robot_x;
extern float robot_y;
extern float robot_yaw;

// OPS-9 串口接收中断处理函数声明
void OPS9_UART_RxCpltCallback(UART_HandleTypeDef *huart);



#endif /* INC_OPS9_H_ */

/*
 * ops9.c
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */
#include "ops9.h"
#include "usart.h" // 需要用到 huart2

// 暴露给外部使用的坐标变量（已转换为你的坐标系：X前，Y左）
float robot_x = 0.0f;
float robot_y = 0.0f;
float robot_yaw = 0.0f;

// 用于 HAL 库单字节接收的缓存
uint8_t ops9_rx_byte;

// 数据解析状态机变量
static uint8_t count = 0;
static uint8_t i = 0;

// 利用共用体直接将24个字节转换为6个浮点数
static union {
    uint8_t data[24];
    float ActVal[6];
} posture;

/*
 * @brief OPS-9 串口单字节接收回调函数
 * @note  请在 HAL_UART_RxCpltCallback 中调用此函数
 */
void OPS9_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // 确认是 OPS-9 所在的串口2
    {
        uint8_t ch = ops9_rx_byte;

        // 状态机解析 (参考官方手册附录)
        switch (count)
        {
            case 0: // 等待帧头第一个字节 0x0D
                if (ch == 0x0D)
                    count++;
                else
                    count = 0;
                break;

            case 1: // 等待帧头第二个字节 0x0A
                if (ch == 0x0A) {
                    i = 0;
                    count++;
                } else if (ch == 0x0D) {
                    ; // 保持状态
                } else {
                    count = 0;
                }
                break;

            case 2: // 接收 24 字节的数据区
                posture.data[i] = ch;
                i++;
                if (i >= 24) {
                    i = 0;
                    count++;
                }
                break;

            case 3: // 等待帧尾第一个字节 0x0A
                if (ch == 0x0A)
                    count++;
                else
                    count = 0;
                break;

            case 4: // 等待帧尾第二个字节 0x0D 并提取数据
                if (ch == 0x0D)
                {
                    // 数据提取：航向角为 ActVal[0]，X为 ActVal[3]，Y为 ActVal[4] [cite: 557]
                    float ops_zangle = posture.ActVal[0];
                    float ops_pos_x  = posture.ActVal[3];
                    float ops_pos_y  = posture.ActVal[4];

                    // === 坐标系转换 (将OPS9的"Y前X右" 转为你的 "X前Y左") ===
                    robot_x = ops_pos_y;    // OPS的Y(前) -> 你的X(前)
                    robot_y = -ops_pos_x;   // OPS的X(右) -> 你的Y(左)
                    robot_yaw = ops_zangle; // 航向角 (假设逆时针为正，如需取反加负号即可)
                }
                count = 0;
                break;

            default:
                count = 0;
                break;
        }

        // 重新开启下一次单字节中断接收
        HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
    }
}




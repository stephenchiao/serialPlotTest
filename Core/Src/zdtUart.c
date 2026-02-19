/*
 * zdtUart.c
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */
#include "zdtUart.h"
#include <string.h>

// 引入底层寄存器定义
#include "stm32f4xx.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
// 静态缓冲区
void ZDT_UART_TransmitDMA(uint8_t *data, uint16_t len)
{
    // 检查串口是否处于忙碌状态，如果忙（上一次还没发完），这次就不发了
    // 这样能防止波形卡死
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart1, data, len);
    }
}

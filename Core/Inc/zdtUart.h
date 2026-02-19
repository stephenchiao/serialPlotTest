/*
 * zdtUart.h
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */

#ifndef INC_ZDTUART_H_
#define INC_ZDTUART_H_

#include "main.h"
#include "usart.h"

// 使用 DMA 发送数据
void ZDT_UART_TransmitDMA(uint8_t *data, uint16_t len);



#endif /* INC_ZDTUART_H_ */

#ifndef RPI_UART_LINK_H
#define RPI_UART_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_link.h"

typedef struct
{
    uint32_t queued_frames;
    uint32_t dropped_frames;
    uint32_t transmit_errors;
    uint32_t receive_errors;
} RpiUartLinkStatistics;

HAL_StatusTypeDef RpiUartLink_Start(UART_HandleTypeDef *huart);
void RpiUartLink_Process(void);
void RpiUartLink_OnRxComplete(UART_HandleTypeDef *huart);
void RpiUartLink_OnError(UART_HandleTypeDef *huart);

HAL_StatusTypeDef RpiUartLink_Send(uint8_t message_type,
                                  const uint8_t *payload,
                                  uint16_t payload_length);

HAL_StatusTypeDef RpiUartLink_SendResponse(uint8_t request_sequence,
                                          uint8_t command,
                                          RpiResponseStatus status,
                                          const uint8_t *data,
                                          uint16_t data_length);

const RpiUartLinkStatistics *RpiUartLink_GetStatistics(void);
const RpiProtocolParser *RpiUartLink_GetParser(void);
uint32_t RpiUartLink_GetLastReceiveTick(void);

#ifdef __cplusplus
}
#endif

#endif /* RPI_UART_LINK_H */

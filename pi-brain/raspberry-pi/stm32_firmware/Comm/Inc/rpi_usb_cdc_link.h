#ifndef RPI_USB_CDC_LINK_H
#define RPI_USB_CDC_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_link.h"

typedef struct
{
    uint32_t received_packets;
    uint32_t received_bytes;
    uint32_t queued_frames;
    uint32_t dropped_frames;
    uint32_t queued_transmits;
    uint32_t transmit_errors;
    uint32_t receive_errors;
} RpiUsbCdcLinkStatistics;

/* 在 MX_USB_DEVICE_Init() 之后调用。 */
HAL_StatusTypeDef RpiUsbCdcLink_Start(void);

/*
 * 在 CDC_Receive_FS() 中调用。此函数只解析并入队，不执行底盘动作。
 * CubeMX 生成的 CDC_Receive_FS() 仍须重新挂载接收缓冲区。
 */
void RpiUsbCdcLink_OnReceive(const uint8_t *data, uint32_t length);

/* 在裸机主循环或唯一通信任务中每 1~5 ms 调用。 */
void RpiUsbCdcLink_Process(void);

HAL_StatusTypeDef RpiUsbCdcLink_Send(uint8_t message_type,
                                    const uint8_t *payload,
                                    uint16_t payload_length);

HAL_StatusTypeDef RpiUsbCdcLink_SendResponse(uint8_t request_sequence,
                                            uint8_t command,
                                            RpiResponseStatus status,
                                            const uint8_t *data,
                                            uint16_t data_length);

const RpiUsbCdcLinkStatistics *RpiUsbCdcLink_GetStatistics(void);
const RpiProtocolParser *RpiUsbCdcLink_GetParser(void);
uint32_t RpiUsbCdcLink_GetLastReceiveTick(void);

#ifdef __cplusplus
}
#endif

#endif /* RPI_USB_CDC_LINK_H */

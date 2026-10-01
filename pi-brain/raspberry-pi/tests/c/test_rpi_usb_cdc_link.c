#include "rpi_usb_cdc_link.h"

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_def.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static USBD_CDC_HandleTypeDef cdc_state;
USBD_HandleTypeDef hUsbDeviceFS = {&cdc_state};
static uint32_t fake_tick = 100U;
static uint8_t transmitted[RPI_PROTOCOL_MAX_FRAME];
static uint16_t transmitted_length;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    if (cdc_state.TxState != 0U)
    {
        return USBD_BUSY;
    }
    memcpy(transmitted, buffer, length);
    transmitted_length = length;
    cdc_state.TxState = 1U;
    return USBD_OK;
}

RpiResponseStatus RpiLink_HandleCommand(uint8_t command,
                                       const uint8_t *data,
                                       uint16_t data_length,
                                       uint8_t *response_data,
                                       uint16_t response_capacity,
                                       uint16_t *response_length)
{
    (void)data;
    (void)response_data;
    (void)response_capacity;
    *response_length = 0U;
    if ((command == RPI_CMD_PING) && (data_length == 0U))
    {
        return RPI_STATUS_OK;
    }
    return RPI_STATUS_UNKNOWN_COMMAND;
}

int main(void)
{
    uint8_t command_payload[] = {RPI_CMD_PING};
    uint8_t command_frame[RPI_PROTOCOL_MAX_FRAME];
    uint16_t command_length;
    RpiProtocolParser response_parser;
    uint16_t index;

    command_length = RpiProtocol_Encode(RPI_MSG_COMMAND,
                                        0x2AU,
                                        command_payload,
                                        sizeof(command_payload),
                                        command_frame,
                                        sizeof(command_frame));
    assert(command_length != 0U);
    assert(RpiUsbCdcLink_Start() == HAL_OK);

    fake_tick = 250U;
    RpiUsbCdcLink_OnReceive(command_frame, 3U);
    RpiUsbCdcLink_OnReceive(&command_frame[3], command_length - 3U);
    RpiUsbCdcLink_Process();

    assert(transmitted_length != 0U);
    RpiProtocol_ParserInit(&response_parser, NULL, NULL);
    for (index = 0U; index < transmitted_length; ++index)
    {
        RpiProtocol_FeedByte(&response_parser, transmitted[index]);
    }
    assert(response_parser.valid_frames == 1U);
    assert(response_parser.frame.message_type == RPI_MSG_RESPONSE);
    assert(response_parser.frame.payload_length == 3U);
    assert(response_parser.frame.payload[0] == 0x2AU);
    assert(response_parser.frame.payload[1] == RPI_CMD_PING);
    assert(response_parser.frame.payload[2] == RPI_STATUS_OK);
    assert(RpiUsbCdcLink_GetLastReceiveTick() == 250U);

    cdc_state.TxState = 0U;
    RpiUsbCdcLink_Process();
    assert(RpiUsbCdcLink_GetStatistics()->received_packets == 2U);
    assert(RpiUsbCdcLink_GetStatistics()->queued_transmits == 1U);
    return 0;
}

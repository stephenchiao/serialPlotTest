#include "rpi_usb_cdc_link.h"

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <stddef.h>

#define RPI_USB_RX_QUEUE_SIZE 4U
#define RPI_USB_TX_QUEUE_SIZE 8U
#define RPI_RESPONSE_PREFIX_SIZE 3U

typedef struct
{
    uint16_t length;
    uint8_t data[RPI_PROTOCOL_MAX_FRAME];
} RpiUsbTxFrame;

static uint8_t s_started;
static uint8_t s_tx_sequence;
static uint8_t s_tx_active;
static RpiProtocolParser s_parser;
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint32_t s_last_receive_tick;
static RpiFrame s_rx_queue[RPI_USB_RX_QUEUE_SIZE];
static RpiUsbTxFrame s_tx_queue[RPI_USB_TX_QUEUE_SIZE];
static RpiUsbCdcLinkStatistics s_statistics;

static void copy_frame(RpiFrame *destination, const RpiFrame *source)
{
    uint16_t index;

    destination->version = source->version;
    destination->message_type = source->message_type;
    destination->sequence = source->sequence;
    destination->payload_length = source->payload_length;
    for (index = 0U; index < source->payload_length; ++index)
    {
        destination->payload[index] = source->payload[index];
    }
}

static void frame_received_from_usb(const RpiFrame *frame, void *context)
{
    uint8_t next_head;

    (void)context;
    next_head = (uint8_t)((s_rx_head + 1U) % RPI_USB_RX_QUEUE_SIZE);
    if (next_head == s_rx_tail)
    {
        ++s_statistics.dropped_frames;
        return;
    }
    copy_frame(&s_rx_queue[s_rx_head], frame);
    s_rx_head = next_head;
    s_last_receive_tick = HAL_GetTick();
    ++s_statistics.queued_frames;
}

static uint8_t dequeue_received_frame(RpiFrame *frame)
{
    uint32_t interrupt_state;

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (s_rx_tail == s_rx_head)
    {
        if (interrupt_state == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }
    copy_frame(frame, &s_rx_queue[s_rx_tail]);
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % RPI_USB_RX_QUEUE_SIZE);
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

static uint8_t usb_cdc_transmit_idle(void)
{
    USBD_CDC_HandleTypeDef *cdc;

    if (hUsbDeviceFS.pClassData == NULL)
    {
        return 0U;
    }
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (cdc->TxState == 0U) ? 1U : 0U;
}

static void service_transmit_queue(void)
{
    uint8_t usb_result;

    if ((s_tx_active != 0U) && (usb_cdc_transmit_idle() != 0U))
    {
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % RPI_USB_TX_QUEUE_SIZE);
        s_tx_active = 0U;
    }
    if ((s_tx_active != 0U) || (s_tx_tail == s_tx_head) ||
        (usb_cdc_transmit_idle() == 0U))
    {
        return;
    }
    usb_result = CDC_Transmit_FS(s_tx_queue[s_tx_tail].data,
                                s_tx_queue[s_tx_tail].length);
    if (usb_result == USBD_OK)
    {
        s_tx_active = 1U;
    }
    else if (usb_result != USBD_BUSY)
    {
        ++s_statistics.transmit_errors;
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % RPI_USB_TX_QUEUE_SIZE);
    }
}

HAL_StatusTypeDef RpiUsbCdcLink_Start(void)
{
    s_started = 1U;
    s_tx_sequence = 0U;
    s_tx_active = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_last_receive_tick = HAL_GetTick();
    s_statistics.received_packets = 0U;
    s_statistics.received_bytes = 0U;
    s_statistics.queued_frames = 0U;
    s_statistics.dropped_frames = 0U;
    s_statistics.queued_transmits = 0U;
    s_statistics.transmit_errors = 0U;
    s_statistics.receive_errors = 0U;
    RpiProtocol_ParserInit(&s_parser, frame_received_from_usb, NULL);
    return HAL_OK;
}

void RpiUsbCdcLink_OnReceive(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if ((s_started == 0U) || ((data == NULL) && (length != 0U)))
    {
        ++s_statistics.receive_errors;
        return;
    }
    ++s_statistics.received_packets;
    s_statistics.received_bytes += length;
    for (index = 0U; index < length; ++index)
    {
        RpiProtocol_FeedByte(&s_parser, data[index]);
    }
}

HAL_StatusTypeDef RpiUsbCdcLink_Send(uint8_t message_type,
                                    const uint8_t *payload,
                                    uint16_t payload_length)
{
    uint8_t next_head;
    uint16_t frame_length;

    if (s_started == 0U)
    {
        return HAL_ERROR;
    }
    next_head = (uint8_t)((s_tx_head + 1U) % RPI_USB_TX_QUEUE_SIZE);
    if (next_head == s_tx_tail)
    {
        ++s_statistics.transmit_errors;
        return HAL_BUSY;
    }
    frame_length = RpiProtocol_Encode(message_type,
                                     s_tx_sequence++,
                                     payload,
                                     payload_length,
                                     s_tx_queue[s_tx_head].data,
                                     sizeof(s_tx_queue[s_tx_head].data));
    if (frame_length == 0U)
    {
        ++s_statistics.transmit_errors;
        return HAL_ERROR;
    }
    s_tx_queue[s_tx_head].length = frame_length;
    s_tx_head = next_head;
    ++s_statistics.queued_transmits;
    service_transmit_queue();
    return HAL_OK;
}

HAL_StatusTypeDef RpiUsbCdcLink_SendResponse(uint8_t request_sequence,
                                            uint8_t command,
                                            RpiResponseStatus status,
                                            const uint8_t *data,
                                            uint16_t data_length)
{
    uint8_t payload[RPI_PROTOCOL_MAX_PAYLOAD];
    uint16_t index;

    if ((data_length > (RPI_PROTOCOL_MAX_PAYLOAD - RPI_RESPONSE_PREFIX_SIZE)) ||
        ((data == NULL) && (data_length != 0U)))
    {
        return HAL_ERROR;
    }
    payload[0] = request_sequence;
    payload[1] = command;
    payload[2] = (uint8_t)status;
    for (index = 0U; index < data_length; ++index)
    {
        payload[RPI_RESPONSE_PREFIX_SIZE + index] = data[index];
    }
    return RpiUsbCdcLink_Send(
        RPI_MSG_RESPONSE,
        payload,
        (uint16_t)(RPI_RESPONSE_PREFIX_SIZE + data_length));
}

void RpiUsbCdcLink_Process(void)
{
    RpiFrame frame;
    uint8_t command;
    uint8_t response_data[RPI_PROTOCOL_MAX_PAYLOAD - RPI_RESPONSE_PREFIX_SIZE];
    uint16_t response_length;
    RpiResponseStatus status;

    service_transmit_queue();
    while (dequeue_received_frame(&frame) != 0U)
    {
        if (frame.message_type != RPI_MSG_COMMAND)
        {
            RpiLink_HandleFrame(&frame);
            continue;
        }
        if (frame.payload_length == 0U)
        {
            (void)RpiUsbCdcLink_SendResponse(frame.sequence,
                                            0U,
                                            RPI_STATUS_INVALID_LENGTH,
                                            NULL,
                                            0U);
            continue;
        }
        command = frame.payload[0];
        response_length = 0U;
        status = RpiLink_HandleCommand(
            command,
            &frame.payload[1],
            (uint16_t)(frame.payload_length - 1U),
            response_data,
            sizeof(response_data),
            &response_length);
        if (response_length > sizeof(response_data))
        {
            status = RPI_STATUS_INTERNAL_ERROR;
            response_length = 0U;
        }
        (void)RpiUsbCdcLink_SendResponse(frame.sequence,
                                        command,
                                        status,
                                        response_data,
                                        response_length);
    }
    service_transmit_queue();
}

__weak void RpiLink_HandleFrame(const RpiFrame *frame)
{
    (void)frame;
}

const RpiUsbCdcLinkStatistics *RpiUsbCdcLink_GetStatistics(void)
{
    return &s_statistics;
}

const RpiProtocolParser *RpiUsbCdcLink_GetParser(void)
{
    return &s_parser;
}

uint32_t RpiUsbCdcLink_GetLastReceiveTick(void)
{
    return s_last_receive_tick;
}

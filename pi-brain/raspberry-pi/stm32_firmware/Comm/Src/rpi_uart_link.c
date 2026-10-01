#include "rpi_uart_link.h"

#include <stddef.h>

#define RPI_RX_QUEUE_SIZE 4U
#define RPI_RESPONSE_PREFIX_SIZE 3U
#define RPI_HAL_TX_TIMEOUT_MS 20U

static UART_HandleTypeDef *s_huart;
static uint8_t s_rx_byte;
static uint8_t s_tx_sequence;
static RpiProtocolParser s_parser;
static volatile uint8_t s_queue_head;
static volatile uint8_t s_queue_tail;
static volatile uint32_t s_last_receive_tick;
static RpiFrame s_queue[RPI_RX_QUEUE_SIZE];
static RpiUartLinkStatistics s_statistics;

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

static void frame_received_from_isr(const RpiFrame *frame, void *context)
{
    uint8_t next_head;

    (void)context;
    next_head = (uint8_t)((s_queue_head + 1U) % RPI_RX_QUEUE_SIZE);
    if (next_head == s_queue_tail)
    {
        ++s_statistics.dropped_frames;
        return;
    }
    copy_frame(&s_queue[s_queue_head], frame);
    s_queue_head = next_head;
    s_last_receive_tick = HAL_GetTick();
    ++s_statistics.queued_frames;
}

static uint8_t dequeue_frame(RpiFrame *frame)
{
    uint32_t interrupt_state;

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (s_queue_tail == s_queue_head)
    {
        if (interrupt_state == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }
    copy_frame(frame, &s_queue[s_queue_tail]);
    s_queue_tail = (uint8_t)((s_queue_tail + 1U) % RPI_RX_QUEUE_SIZE);
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

HAL_StatusTypeDef RpiUartLink_Start(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }
    s_huart = huart;
    s_tx_sequence = 0U;
    s_queue_head = 0U;
    s_queue_tail = 0U;
    s_last_receive_tick = HAL_GetTick();
    s_statistics.queued_frames = 0U;
    s_statistics.dropped_frames = 0U;
    s_statistics.transmit_errors = 0U;
    s_statistics.receive_errors = 0U;
    RpiProtocol_ParserInit(&s_parser, frame_received_from_isr, NULL);
    return HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
}

void RpiUartLink_OnRxComplete(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart))
    {
        return;
    }
    RpiProtocol_FeedByte(&s_parser, s_rx_byte);
    if (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U) != HAL_OK)
    {
        ++s_statistics.receive_errors;
    }
}

void RpiUartLink_OnError(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart))
    {
        return;
    }
    ++s_statistics.receive_errors;
    RpiProtocol_ParserReset(&s_parser);
    (void)HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U) != HAL_OK)
    {
        ++s_statistics.receive_errors;
    }
}

HAL_StatusTypeDef RpiUartLink_Send(uint8_t message_type,
                                  const uint8_t *payload,
                                  uint16_t payload_length)
{
    uint8_t frame_buffer[RPI_PROTOCOL_MAX_FRAME];
    uint16_t frame_length;
    HAL_StatusTypeDef result;

    if (s_huart == NULL)
    {
        return HAL_ERROR;
    }
    frame_length = RpiProtocol_Encode(message_type,
                                     s_tx_sequence++,
                                     payload,
                                     payload_length,
                                     frame_buffer,
                                     sizeof(frame_buffer));
    if (frame_length == 0U)
    {
        return HAL_ERROR;
    }
    result = HAL_UART_Transmit(s_huart,
                              frame_buffer,
                              frame_length,
                              RPI_HAL_TX_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        ++s_statistics.transmit_errors;
    }
    return result;
}

HAL_StatusTypeDef RpiUartLink_SendResponse(uint8_t request_sequence,
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
    return RpiUartLink_Send(RPI_MSG_RESPONSE,
                           payload,
                           (uint16_t)(RPI_RESPONSE_PREFIX_SIZE + data_length));
}

void RpiUartLink_Process(void)
{
    RpiFrame frame;
    uint8_t command;
    uint8_t response_data[RPI_PROTOCOL_MAX_PAYLOAD - RPI_RESPONSE_PREFIX_SIZE];
    uint16_t response_length;
    RpiResponseStatus status;

    while (dequeue_frame(&frame) != 0U)
    {
        if (frame.message_type != RPI_MSG_COMMAND)
        {
            RpiLink_HandleFrame(&frame);
            continue;
        }
        if (frame.payload_length == 0U)
        {
            (void)RpiUartLink_SendResponse(frame.sequence,
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
        (void)RpiUartLink_SendResponse(frame.sequence,
                                      command,
                                      status,
                                      response_data,
                                      response_length);
    }
}

__weak RpiResponseStatus RpiLink_HandleCommand(uint8_t command,
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
    if (command == RPI_CMD_PING)
    {
        return (data_length == 0U) ? RPI_STATUS_OK : RPI_STATUS_INVALID_LENGTH;
    }
    return RPI_STATUS_UNKNOWN_COMMAND;
}

__weak void RpiLink_HandleFrame(const RpiFrame *frame)
{
    (void)frame;
}

const RpiUartLinkStatistics *RpiUartLink_GetStatistics(void)
{
    return &s_statistics;
}

const RpiProtocolParser *RpiUartLink_GetParser(void)
{
    return &s_parser;
}

uint32_t RpiUartLink_GetLastReceiveTick(void)
{
    return s_last_receive_tick;
}

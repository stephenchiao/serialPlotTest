#include "rpi_protocol.h"

#include <stddef.h>

#define RPI_SOF_1 0xA5U
#define RPI_SOF_2 0x5AU
#define RPI_HEADER_BODY_SIZE 5U
#define RPI_FIXED_FRAME_SIZE 9U

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= (uint16_t)byte << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

uint16_t RpiProtocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    for (index = 0U; index < length; ++index)
    {
        crc = crc16_update(crc, data[index]);
    }
    return crc;
}

uint16_t RpiProtocol_Encode(uint8_t message_type,
                            uint8_t sequence,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *output,
                            uint16_t output_capacity)
{
    uint16_t crc;
    uint16_t index;
    uint16_t total_length = (uint16_t)(RPI_FIXED_FRAME_SIZE + payload_length);

    if ((output == NULL) ||
        ((payload == NULL) && (payload_length != 0U)) ||
        (payload_length > RPI_PROTOCOL_MAX_PAYLOAD) ||
        (output_capacity < total_length))
    {
        return 0U;
    }

    output[0] = RPI_SOF_1;
    output[1] = RPI_SOF_2;
    output[2] = RPI_PROTOCOL_VERSION;
    output[3] = message_type;
    output[4] = sequence;
    output[5] = (uint8_t)(payload_length & 0xFFU);
    output[6] = (uint8_t)(payload_length >> 8);

    for (index = 0U; index < payload_length; ++index)
    {
        output[7U + index] = payload[index];
    }

    crc = RpiProtocol_Crc16(&output[2], (uint16_t)(5U + payload_length));
    output[7U + payload_length] = (uint8_t)(crc & 0xFFU);
    output[8U + payload_length] = (uint8_t)(crc >> 8);
    return total_length;
}

void RpiProtocol_ParserReset(RpiProtocolParser *parser)
{
    if (parser == NULL)
    {
        return;
    }
    parser->state = RPI_PARSE_SOF_1;
    parser->header_index = 0U;
    parser->payload_index = 0U;
    parser->received_crc = 0U;
    parser->running_crc = 0xFFFFU;
}

void RpiProtocol_ParserInit(RpiProtocolParser *parser,
                            RpiFrameCallback callback,
                            void *callback_context)
{
    if (parser == NULL)
    {
        return;
    }
    parser->valid_frames = 0U;
    parser->crc_errors = 0U;
    parser->format_errors = 0U;
    parser->callback = callback;
    parser->callback_context = callback_context;
    RpiProtocol_ParserReset(parser);
}

void RpiProtocol_FeedByte(RpiProtocolParser *parser, uint8_t byte)
{
    uint16_t payload_length;

    if (parser == NULL)
    {
        return;
    }

    switch (parser->state)
    {
        case RPI_PARSE_SOF_1:
            if (byte == RPI_SOF_1)
            {
                parser->state = RPI_PARSE_SOF_2;
            }
            break;

        case RPI_PARSE_SOF_2:
            if (byte == RPI_SOF_2)
            {
                parser->state = RPI_PARSE_HEADER;
                parser->header_index = 0U;
                parser->running_crc = 0xFFFFU;
            }
            else if (byte != RPI_SOF_1)
            {
                parser->state = RPI_PARSE_SOF_1;
            }
            break;

        case RPI_PARSE_HEADER:
            parser->header[parser->header_index++] = byte;
            parser->running_crc = crc16_update(parser->running_crc, byte);
            if (parser->header_index == RPI_HEADER_BODY_SIZE)
            {
                payload_length = (uint16_t)parser->header[3] |
                                 ((uint16_t)parser->header[4] << 8);
                if ((parser->header[0] != RPI_PROTOCOL_VERSION) ||
                    (payload_length > RPI_PROTOCOL_MAX_PAYLOAD))
                {
                    ++parser->format_errors;
                    RpiProtocol_ParserReset(parser);
                    break;
                }
                parser->frame.version = parser->header[0];
                parser->frame.message_type = parser->header[1];
                parser->frame.sequence = parser->header[2];
                parser->frame.payload_length = payload_length;
                parser->payload_index = 0U;
                parser->state = (payload_length == 0U) ?
                                RPI_PARSE_CRC_LOW : RPI_PARSE_PAYLOAD;
            }
            break;

        case RPI_PARSE_PAYLOAD:
            parser->frame.payload[parser->payload_index++] = byte;
            parser->running_crc = crc16_update(parser->running_crc, byte);
            if (parser->payload_index == parser->frame.payload_length)
            {
                parser->state = RPI_PARSE_CRC_LOW;
            }
            break;

        case RPI_PARSE_CRC_LOW:
            parser->received_crc = byte;
            parser->state = RPI_PARSE_CRC_HIGH;
            break;

        case RPI_PARSE_CRC_HIGH:
            parser->received_crc |= (uint16_t)byte << 8;
            if (parser->received_crc == parser->running_crc)
            {
                ++parser->valid_frames;
                if (parser->callback != NULL)
                {
                    parser->callback(&parser->frame, parser->callback_context);
                }
            }
            else
            {
                ++parser->crc_errors;
            }
            RpiProtocol_ParserReset(parser);
            break;

        default:
            ++parser->format_errors;
            RpiProtocol_ParserReset(parser);
            break;
    }
}


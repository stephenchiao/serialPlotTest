#ifndef RPI_PROTOCOL_H
#define RPI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RPI_PROTOCOL_VERSION       1U
#define RPI_PROTOCOL_MAX_PAYLOAD   128U
#define RPI_PROTOCOL_MAX_FRAME     (2U + 5U + RPI_PROTOCOL_MAX_PAYLOAD + 2U)

typedef enum
{
    RPI_MSG_COMMAND   = 0x10,
    RPI_MSG_RESPONSE  = 0x11,
    RPI_MSG_TELEMETRY = 0x20,
    RPI_MSG_HEARTBEAT = 0x21,
    RPI_MSG_EVENT     = 0x22
} RpiMessageType;

typedef struct
{
    uint8_t version;
    uint8_t message_type;
    uint8_t sequence;
    uint16_t payload_length;
    uint8_t payload[RPI_PROTOCOL_MAX_PAYLOAD];
} RpiFrame;

typedef void (*RpiFrameCallback)(const RpiFrame *frame, void *context);

typedef enum
{
    RPI_PARSE_SOF_1 = 0,
    RPI_PARSE_SOF_2,
    RPI_PARSE_HEADER,
    RPI_PARSE_PAYLOAD,
    RPI_PARSE_CRC_LOW,
    RPI_PARSE_CRC_HIGH
} RpiParserState;

typedef struct
{
    RpiParserState state;
    uint8_t header[5];
    uint8_t header_index;
    uint16_t payload_index;
    uint16_t received_crc;
    uint16_t running_crc;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    RpiFrame frame;
    RpiFrameCallback callback;
    void *callback_context;
} RpiProtocolParser;

uint16_t RpiProtocol_Crc16(const uint8_t *data, uint16_t length);

uint16_t RpiProtocol_Encode(uint8_t message_type,
                            uint8_t sequence,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *output,
                            uint16_t output_capacity);

void RpiProtocol_ParserInit(RpiProtocolParser *parser,
                            RpiFrameCallback callback,
                            void *callback_context);

void RpiProtocol_ParserReset(RpiProtocolParser *parser);

void RpiProtocol_FeedByte(RpiProtocolParser *parser, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* RPI_PROTOCOL_H */


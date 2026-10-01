#include "rpi_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static RpiFrame received_frame;
static uint8_t callback_count;

static void on_frame(const RpiFrame *frame, void *context)
{
    (void)context;
    received_frame = *frame;
    ++callback_count;
}

int main(void)
{
    static const uint8_t crc_vector[] = "123456789";
    static const uint8_t payload[] = {0x10U, 0x78U, 0x00U, 0xE2U,
                                      0xFFU, 0xFAU, 0x00U};
    static const uint8_t expected[] = {
        0xA5U, 0x5AU, 0x01U, 0x10U, 0x25U, 0x07U, 0x00U, 0x10U,
        0x78U, 0x00U, 0xE2U, 0xFFU, 0xFAU, 0x00U, 0x29U, 0xBDU
    };
    uint8_t output[RPI_PROTOCOL_MAX_FRAME];
    uint16_t output_length;
    uint16_t index;
    RpiProtocolParser parser;

    assert(RpiProtocol_Crc16(crc_vector, 9U) == 0x29B1U);
    output_length = RpiProtocol_Encode(RPI_MSG_COMMAND,
                                      0x25U,
                                      payload,
                                      sizeof(payload),
                                      output,
                                      sizeof(output));
    assert(output_length == sizeof(expected));
    assert(memcmp(output, expected, sizeof(expected)) == 0);

    RpiProtocol_ParserInit(&parser, on_frame, NULL);
    RpiProtocol_FeedByte(&parser, 0x00U);
    for (index = 0U; index < output_length; ++index)
    {
        RpiProtocol_FeedByte(&parser, output[index]);
    }
    assert(callback_count == 1U);
    assert(received_frame.message_type == RPI_MSG_COMMAND);
    assert(received_frame.sequence == 0x25U);
    assert(received_frame.payload_length == sizeof(payload));
    assert(memcmp(received_frame.payload, payload, sizeof(payload)) == 0);
    assert(parser.valid_frames == 1U);
    return 0;
}

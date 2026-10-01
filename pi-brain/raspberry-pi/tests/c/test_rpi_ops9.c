#include "rpi_ops9.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint8_t captured_type;
static uint8_t captured_payload[32];
static uint16_t captured_length;

HAL_StatusTypeDef RpiUsbCdcLink_Send(uint8_t message_type,
                                    const uint8_t *payload,
                                    uint16_t payload_length)
{
    captured_type = message_type;
    captured_length = payload_length;
    memcpy(captured_payload, payload, payload_length);
    return HAL_OK;
}

int main(void)
{
    static const uint8_t expected[] = {
        0x01U,
        0x85U, 0xFFU, 0xFFU, 0xFFU,
        0xC8U, 0x01U, 0x00U, 0x00U,
        0xDDU, 0xF9U, 0xFFU, 0xFFU,
        0x78U, 0x56U, 0x34U, 0x12U,
        87U,
        0x07U
    };

    assert(RpiOps9_SendPose(-123,
                            456,
                            -1571,
                            0x12345678U,
                            87U,
                            RPI_OPS9_STATUS_VALID |
                            RPI_OPS9_STATUS_CALIBRATED |
                            RPI_OPS9_STATUS_CONTACT_OK) == HAL_OK);
    assert(captured_type == RPI_MSG_TELEMETRY);
    assert(captured_length == sizeof(expected));
    assert(memcmp(captured_payload, expected, sizeof(expected)) == 0);
    assert(RpiOps9_SendPose(0, 0, 0, 0U, 101U, 0U) == HAL_ERROR);
    return 0;
}

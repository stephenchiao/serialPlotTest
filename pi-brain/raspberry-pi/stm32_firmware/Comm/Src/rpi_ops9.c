#include "rpi_ops9.h"

#include <stddef.h>

#define RPI_OPS9_PAYLOAD_SIZE 19U

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)((value >> 24) & 0xFFU);
}

HAL_StatusTypeDef RpiOps9_SendPose(int32_t x_mm,
                                  int32_t y_mm,
                                  int32_t yaw_mrad,
                                  uint32_t timestamp_ms,
                                  uint8_t quality,
                                  uint8_t status_flags)
{
    uint8_t payload[RPI_OPS9_PAYLOAD_SIZE];

    if (quality > 100U)
    {
        return HAL_ERROR;
    }
    payload[0] = RPI_TELEMETRY_OPS9_POSE;
    write_u32_le(&payload[1], (uint32_t)x_mm);
    write_u32_le(&payload[5], (uint32_t)y_mm);
    write_u32_le(&payload[9], (uint32_t)yaw_mrad);
    write_u32_le(&payload[13], timestamp_ms);
    payload[17] = quality;
    payload[18] = status_flags;
    return RpiTransport_Send(RPI_MSG_TELEMETRY, payload, sizeof(payload));
}

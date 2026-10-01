#ifndef RPI_LINK_H
#define RPI_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "rpi_protocol.h"

typedef enum
{
    RPI_STATUS_OK = 0x00,
    RPI_STATUS_UNKNOWN_COMMAND = 0x01,
    RPI_STATUS_INVALID_LENGTH = 0x02,
    RPI_STATUS_INVALID_ARGUMENT = 0x03,
    RPI_STATUS_BUSY = 0x04,
    RPI_STATUS_INTERNAL_ERROR = 0x05
} RpiResponseStatus;

typedef enum
{
    RPI_CMD_PING = 0x01,
    RPI_CMD_STOP_ALL = 0x02,
    RPI_CMD_SET_CHASSIS_VELOCITY = 0x10,
    RPI_CMD_SET_SERVO_ANGLE = 0x20,
    RPI_CMD_QUERY_STATUS = 0x30,
    RPI_CMD_SET_TASK_CODE = 0x40,
    RPI_CMD_SET_POSE_GOAL = 0x80,
    RPI_CMD_CANCEL_POSE_GOAL = 0x81,
    RPI_CMD_QUERY_POSE_GOAL = 0x82
} RpiCommand;

/* 由 rpi_app_commands.c 实现，USB CDC 与 UART 后端共同调用。 */
RpiResponseStatus RpiLink_HandleCommand(uint8_t command,
                                       const uint8_t *data,
                                       uint16_t data_length,
                                       uint8_t *response_data,
                                       uint16_t response_capacity,
                                       uint16_t *response_length);

/* 非 COMMAND 帧（例如 HEARTBEAT）由此可选回调处理。 */
void RpiLink_HandleFrame(const RpiFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* RPI_LINK_H */

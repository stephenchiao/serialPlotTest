#ifndef RPI_MATERIAL_VISION_H
#define RPI_MATERIAL_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Independent of the old v1 transport. Dispatch command 0x85 from the v2 task. */
#define RPI_CMD_UPDATE_MATERIAL_VISION 0x85U
#define RPI_CAP_MATERIAL_VISION        0x40UL
#define RPI_MATERIAL_DATA_LENGTH       47U
#define RPI_MATERIAL_NO_CLASS          0xFFFFU
#define RPI_MATERIAL_VISIBLE           0x01U
#define RPI_MATERIAL_CONFIRMED         0x02U
#define RPI_MATERIAL_BACKEND_COLOR     1U
#define RPI_MATERIAL_BACKEND_MODEL     2U

typedef enum
{
    RPI_MATERIAL_SEARCHING = 0,
    RPI_MATERIAL_CONFIRMING = 1,
    RPI_MATERIAL_TRACKING = 2,
    RPI_MATERIAL_MODEL_NOT_CONFIGURED = 3,
    RPI_MATERIAL_MAPPING_NOT_CONFIGURED = 4,
    RPI_MATERIAL_UNMAPPED_CLASS = 5,
    RPI_MATERIAL_TARGET_NOT_MAPPED = 6,
    RPI_MATERIAL_TARGET_NOT_FOUND = 7,
    RPI_MATERIAL_AMBIGUOUS = 8,
    RPI_MATERIAL_INFERENCE_ERROR = 9,
    RPI_MATERIAL_CAMERA_ERROR = 10,
    RPI_MATERIAL_STOPPED = 11,
    RPI_MATERIAL_HOLD = 12,
    RPI_MATERIAL_STALE = 13
} RpiMaterialStatus;

/* Same numeric statuses as v2 RESPONSE; OK means data accepted, not action done. */
typedef enum
{
    RPI_MATERIAL_RX_OK = 0,
    RPI_MATERIAL_RX_INVALID_LENGTH = 2,
    RPI_MATERIAL_RX_INVALID_ARGUMENT = 3
} RpiMaterialReceiveResult;

typedef struct
{
    uint8_t schema_version;
    uint8_t camera_num;
    uint8_t backend;
    uint8_t status;
    uint8_t flags;
    uint32_t session_id;
    uint32_t frame_id;
    uint32_t capture_tick_ms; /* Pi clock; never compare this with HAL_GetTick(). */
    uint16_t valid_for_ms;
    uint16_t target_material_code;
    uint16_t material_code; /* Color backend: configured color/material ID. */
    uint16_t class_id;
    uint16_t confidence_permille;
    uint16_t frame_width;
    uint16_t frame_height;
    uint16_t center_x;
    uint16_t center_y;
    int16_t offset_x_tenths; /* 0.1 pixel, positive image-right; NOT millimetres. */
    int16_t offset_y_tenths; /* 0.1 pixel, positive image-down. */
    uint16_t box_x;
    uint16_t box_y;
    uint16_t box_width;
    uint16_t box_height;
} RpiMaterialObservation;

typedef struct
{
    RpiMaterialObservation latest;
    uint32_t received_tick_ms;
    uint8_t has_sample;
} RpiMaterialReceiver;

/* Reset on binary session start/end, STOP_ALL and host/USB faults. */
void RpiMaterialReceiver_Reset(RpiMaterialReceiver *receiver);

/* Call only from the main loop / communications task after frame CRC validation. */
RpiMaterialReceiveResult RpiMaterialReceiver_Receive(RpiMaterialReceiver *receiver,
                                                     const uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t now_ms);

/* Fresh sample for diagnostics; invalid states have zero flags and coordinates. */
uint8_t RpiMaterialReceiver_GetLatest(const RpiMaterialReceiver *receiver,
                                     uint32_t now_ms,
                                     RpiMaterialObservation *output);

/* Confirmed fresh target, not an alignment/grasp permit. STM32 applies its gates. */
uint8_t RpiMaterialReceiver_GetTarget(const RpiMaterialReceiver *receiver,
                                     uint32_t now_ms,
                                     RpiMaterialObservation *output);

#ifdef __cplusplus
}
#endif

#endif

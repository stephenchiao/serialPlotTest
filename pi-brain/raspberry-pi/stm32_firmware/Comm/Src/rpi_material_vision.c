#include "rpi_material_vision.h"

#include <stddef.h>
#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t read_i16(const uint8_t *data)
{
    uint16_t value = read_u16(data);
    return (int16_t)((value <= 0x7FFFU) ? (int32_t)value : (int32_t)value - 65536L);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint8_t valid_observation(const RpiMaterialObservation *p)
{
    uint8_t visible = (uint8_t)(p->flags & RPI_MATERIAL_VISIBLE);
    uint8_t confirmed = (uint8_t)(p->flags & RPI_MATERIAL_CONFIRMED);

    if ((p->schema_version != 1U) || (p->camera_num != 0U) ||
        ((p->backend != RPI_MATERIAL_BACKEND_COLOR) && (p->backend != RPI_MATERIAL_BACKEND_MODEL)) ||
        (p->status > RPI_MATERIAL_STALE) || ((p->flags & ~3U) != 0U) ||
        (p->session_id == 0U) || (p->frame_id == 0U) ||
        (p->valid_for_ms == 0U) || (p->valid_for_ms > 1000U) ||
        (p->frame_width == 0U) || (p->frame_height == 0U) ||
        (p->confidence_permille > 1000U))
    {
        return 0U;
    }
    if (visible != 0U)
    {
        if (((p->status != RPI_MATERIAL_CONFIRMING) && (p->status != RPI_MATERIAL_TRACKING)) ||
            ((confirmed != 0U) != (p->status == RPI_MATERIAL_TRACKING)) ||
            (p->material_code == 0U) ||
            ((p->target_material_code != 0U) && (p->target_material_code != p->material_code)) ||
            (p->center_x >= p->frame_width) || (p->center_y >= p->frame_height) ||
            (p->box_width == 0U) || (p->box_height == 0U) ||
            ((uint32_t)p->box_x + p->box_width > p->frame_width) ||
            ((uint32_t)p->box_y + p->box_height > p->frame_height) ||
            ((p->backend == RPI_MATERIAL_BACKEND_MODEL) && (p->class_id == RPI_MATERIAL_NO_CLASS)) ||
            ((p->backend == RPI_MATERIAL_BACKEND_COLOR) && (p->class_id != RPI_MATERIAL_NO_CLASS)))
        {
            return 0U;
        }
    }
    else if ((p->flags != 0U) || (p->status == RPI_MATERIAL_CONFIRMING) ||
             (p->status == RPI_MATERIAL_TRACKING) || (p->material_code != 0U) ||
             (p->class_id != RPI_MATERIAL_NO_CLASS) || (p->confidence_permille != 0U) ||
             (p->center_x != 0U) || (p->center_y != 0U) ||
             (p->offset_x_tenths != 0) || (p->offset_y_tenths != 0) ||
             (p->box_x != 0U) || (p->box_y != 0U) ||
             (p->box_width != 0U) || (p->box_height != 0U))
    {
        return 0U;
    }
    return 1U;
}

void RpiMaterialReceiver_Reset(RpiMaterialReceiver *receiver)
{
    if (receiver != NULL)
    {
        memset(receiver, 0, sizeof(*receiver));
    }
}

RpiMaterialReceiveResult RpiMaterialReceiver_Receive(RpiMaterialReceiver *receiver,
                                                     const uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t now_ms)
{
    RpiMaterialObservation p;
    uint32_t delta;

    if ((receiver == NULL) || (data == NULL) || (length != RPI_MATERIAL_DATA_LENGTH))
    {
        return RPI_MATERIAL_RX_INVALID_LENGTH;
    }
    /* Decode field by field: never memcpy the wire into a padded C struct. */
    memset(&p, 0, sizeof(p));
    p.schema_version = data[0];
    p.camera_num = data[1];
    p.backend = data[2];
    p.status = data[3];
    p.flags = data[4];
    p.session_id = read_u32(&data[5]);
    p.frame_id = read_u32(&data[9]);
    p.capture_tick_ms = read_u32(&data[13]);
    p.valid_for_ms = read_u16(&data[17]);
    p.target_material_code = read_u16(&data[19]);
    p.material_code = read_u16(&data[21]);
    p.class_id = read_u16(&data[23]);
    p.confidence_permille = read_u16(&data[25]);
    p.frame_width = read_u16(&data[27]);
    p.frame_height = read_u16(&data[29]);
    p.center_x = read_u16(&data[31]);
    p.center_y = read_u16(&data[33]);
    p.offset_x_tenths = read_i16(&data[35]);
    p.offset_y_tenths = read_i16(&data[37]);
    p.box_x = read_u16(&data[39]);
    p.box_y = read_u16(&data[41]);
    p.box_width = read_u16(&data[43]);
    p.box_height = read_u16(&data[45]);
    if (valid_observation(&p) == 0U)
    {
        return RPI_MATERIAL_RX_INVALID_ARGUMENT;
    }
    if (receiver->has_sample != 0U)
    {
        /* A new host session must explicitly reset this receiver first. */
        delta = p.frame_id - receiver->latest.frame_id;
        if ((p.session_id != receiver->latest.session_id) ||
            (delta == 0U) || (delta >= 0x80000000UL))
        {
            return RPI_MATERIAL_RX_INVALID_ARGUMENT;
        }
    }
    receiver->latest = p;
    receiver->received_tick_ms = now_ms;
    receiver->has_sample = 1U;
    return RPI_MATERIAL_RX_OK;
}

uint8_t RpiMaterialReceiver_GetLatest(const RpiMaterialReceiver *receiver,
                                     uint32_t now_ms,
                                     RpiMaterialObservation *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
    }
    if ((receiver == NULL) || (output == NULL) || (receiver->has_sample == 0U) ||
        ((uint32_t)(now_ms - receiver->received_tick_ms) >= receiver->latest.valid_for_ms))
    {
        return 0U;
    }
    *output = receiver->latest;
    return 1U;
}

uint8_t RpiMaterialReceiver_GetTarget(const RpiMaterialReceiver *receiver,
                                     uint32_t now_ms,
                                     RpiMaterialObservation *output)
{
    if (RpiMaterialReceiver_GetLatest(receiver, now_ms, output) == 0U)
    {
        return 0U;
    }
    if ((output->status != RPI_MATERIAL_TRACKING) ||
        ((output->flags & 3U) != (RPI_MATERIAL_VISIBLE | RPI_MATERIAL_CONFIRMED)))
    {
        memset(output, 0, sizeof(*output));
        return 0U;
    }
    return 1U;
}

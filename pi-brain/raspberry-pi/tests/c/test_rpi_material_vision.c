#include "rpi_material_vision.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

int main(void)
{
    uint8_t data[RPI_MATERIAL_DATA_LENGTH];
    uint8_t invalid[RPI_MATERIAL_DATA_LENGTH];
    RpiMaterialReceiver receiver;
    RpiMaterialObservation output;

#ifdef _WIN32
    (void)_setmode(_fileno(stdin), _O_BINARY);
#endif
    assert(fread(data, 1U, sizeof(data), stdin) == sizeof(data));
    RpiMaterialReceiver_Reset(&receiver);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 5000U) == RPI_MATERIAL_RX_OK);
    assert(RpiMaterialReceiver_GetTarget(&receiver, 5001U, &output) == 1U);
    assert(output.session_id == 0x12345678UL);
    assert(output.frame_id == 1U);
    assert(output.capture_tick_ms == 10000U); /* Not STM32 tick. */
    assert(output.target_material_code == 7U && output.material_code == 7U);
    assert(output.class_id == 0U && output.confidence_permille == 920U);
    assert(output.frame_width == 640U && output.frame_height == 480U);
    assert(output.center_x == 350U && output.center_y == 225U);
    assert(output.offset_x_tenths == 300 && output.offset_y_tenths == -150);
    assert(output.box_x == 330U && output.box_y == 205U);
    assert(output.box_width == 40U && output.box_height == 40U);
    assert(RpiMaterialReceiver_GetTarget(&receiver, 5249U, &output) == 1U);
    assert(RpiMaterialReceiver_GetTarget(&receiver, 5250U, &output) == 0U);
    assert(output.flags == 0U && output.center_x == 0U);

    /* Duplicate frames must not refresh freshness. */
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 5300U) == RPI_MATERIAL_RX_INVALID_ARGUMENT);
    assert(receiver.received_tick_ms == 5000U);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data) - 1U, 5000U) == RPI_MATERIAL_RX_INVALID_LENGTH);
    memcpy(invalid, data, sizeof(invalid));
    invalid[4] = 7U;
    assert(RpiMaterialReceiver_Receive(&receiver, invalid, sizeof(invalid), 5000U) == RPI_MATERIAL_RX_INVALID_ARGUMENT);
    memcpy(invalid, data, sizeof(invalid));
    invalid[25] = 0xFFU;
    invalid[26] = 0xFFU;
    assert(RpiMaterialReceiver_Receive(&receiver, invalid, sizeof(invalid), 5000U) == RPI_MATERIAL_RX_INVALID_ARGUMENT);
    memcpy(invalid, data, sizeof(invalid));
    invalid[19] = 8U; /* Wrong task target. */
    assert(RpiMaterialReceiver_Receive(&receiver, invalid, sizeof(invalid), 5000U) == RPI_MATERIAL_RX_INVALID_ARGUMENT);

    /* Visible but not confirmed: diagnostics yes, target for control no. */
    write_u32(&data[9], 2U);
    data[3] = RPI_MATERIAL_CONFIRMING;
    data[4] = RPI_MATERIAL_VISIBLE;
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 5400U) == RPI_MATERIAL_RX_OK);
    assert(RpiMaterialReceiver_GetLatest(&receiver, 5401U, &output) == 1U);
    assert(RpiMaterialReceiver_GetTarget(&receiver, 5401U, &output) == 0U);
    assert(output.flags == 0U);

    /* Lost target clears the previous position immediately. */
    write_u32(&data[9], 3U);
    data[3] = RPI_MATERIAL_TARGET_NOT_FOUND;
    data[4] = 0U;
    memset(&data[21], 0, sizeof(data) - 21U);
    data[23] = 0xFFU;
    data[24] = 0xFFU;
    data[27] = 0x80U; data[28] = 0x02U; /* width = 640 */
    data[29] = 0xE0U; data[30] = 0x01U; /* height = 480 */
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 5410U) == RPI_MATERIAL_RX_OK);
    assert(RpiMaterialReceiver_GetLatest(&receiver, 5411U, &output) == 1U);
    assert(output.status == RPI_MATERIAL_TARGET_NOT_FOUND && output.flags == 0U);
    assert(RpiMaterialReceiver_GetTarget(&receiver, 5411U, &output) == 0U);

    /* Tick and frame counters may wrap. Different visual sessions require reset. */
    write_u32(&data[9], 0xFFFFFFFFUL);
    RpiMaterialReceiver_Reset(&receiver);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 0xFFFFFFF0UL) == RPI_MATERIAL_RX_OK);
    assert(RpiMaterialReceiver_GetLatest(&receiver, 0x10U, &output) == 1U);
    write_u32(&data[9], 1U);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 0x10U) == RPI_MATERIAL_RX_OK);
    write_u32(&data[5], 42U);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 0x11U) == RPI_MATERIAL_RX_INVALID_ARGUMENT);
    RpiMaterialReceiver_Reset(&receiver);
    assert(RpiMaterialReceiver_Receive(&receiver, data, sizeof(data), 0x11U) == RPI_MATERIAL_RX_OK);
    assert(RpiMaterialReceiver_GetLatest(&receiver, 0x11U, NULL) == 0U);
    return 0;
}

#include "serialPlot.h"
#include "zdtUart.h"
#include <string.h>

void SerialPlot_SendFloats(float *data, uint8_t num_channels) {
    uint8_t tx_buffer[32];
    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0xBB;
    memset(&tx_buffer[2], 0, 6);
    memcpy(&tx_buffer[8], data, num_channels * sizeof(float));
    uint16_t total_len = 8 + (num_channels * sizeof(float));
    ZDT_UART_TransmitDMA(tx_buffer, total_len);
}

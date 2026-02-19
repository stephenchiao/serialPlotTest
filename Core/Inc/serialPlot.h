/*
 * serialPlot.h
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */

#ifndef INC_SERIALPLOT_H_
#define INC_SERIALPLOT_H_

#include <stdint.h>

// 发送 N 个浮点数到 SerialPlot
void SerialPlot_SendFloats(float *data, uint8_t num_channels);



#endif /* INC_SERIALPLOT_H_ */

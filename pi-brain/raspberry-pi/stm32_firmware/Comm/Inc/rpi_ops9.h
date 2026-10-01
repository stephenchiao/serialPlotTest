#ifndef RPI_OPS9_H
#define RPI_OPS9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_transport.h"

#define RPI_TELEMETRY_OPS9_POSE 0x01U
#define RPI_OPS9_STATUS_VALID 0x01U
#define RPI_OPS9_STATUS_CALIBRATED 0x02U
#define RPI_OPS9_STATUS_CONTACT_OK 0x04U

/*
 * 由 OPS9 驱动或采集任务以固定周期调用。坐标单位 mm，航向单位 mrad，
 * quality 范围 0~100，timestamp_ms 建议直接使用 HAL_GetTick()。
 */
HAL_StatusTypeDef RpiOps9_SendPose(int32_t x_mm,
                                  int32_t y_mm,
                                  int32_t yaw_mrad,
                                  uint32_t timestamp_ms,
                                  uint8_t quality,
                                  uint8_t status_flags);

#ifdef __cplusplus
}
#endif

#endif /* RPI_OPS9_H */

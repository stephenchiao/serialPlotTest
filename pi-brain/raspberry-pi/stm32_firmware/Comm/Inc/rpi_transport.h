#ifndef RPI_TRANSPORT_H
#define RPI_TRANSPORT_H

/* 默认使用 STM32 Type-C 原生 USB CDC；可在编译器中改为 UART。 */
#define RPI_TRANSPORT_USB_CDC 1
#define RPI_TRANSPORT_UART    2

#ifndef RPI_TRANSPORT_BACKEND
#define RPI_TRANSPORT_BACKEND RPI_TRANSPORT_USB_CDC
#endif

#if RPI_TRANSPORT_BACKEND == RPI_TRANSPORT_USB_CDC
#include "rpi_usb_cdc_link.h"
#define RpiTransport_Send RpiUsbCdcLink_Send
#elif RPI_TRANSPORT_BACKEND == RPI_TRANSPORT_UART
#include "rpi_uart_link.h"
#define RpiTransport_Send RpiUartLink_Send
#else
#error "Unsupported RPI_TRANSPORT_BACKEND"
#endif

#endif /* RPI_TRANSPORT_H */

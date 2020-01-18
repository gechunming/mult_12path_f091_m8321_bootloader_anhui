#ifndef _USER_USART2_H_
#define _USER_USART2_H_

#include "stm32f0xx.h"

#define UART_FRAME_PACK_START  0x7E
#define UART_FRAME_PACK_TX_START 0x55
#define UART_FRAME_PACK_RX_START 0xAA
#define UART_FRAME_PACK_END 0x0d

#define UART_FRAME_CMD_SLIDEDOOR_SET 0x02
#define UART_FRAME_CMD_SLIDEDOOR_OPEN_GET_STATUS 0x03
#define UART_FRAME_CMD_HIGH_GET_STATUS  0x04
#define UART_FRAME_CMD_GET_TEMP				0x05
#define UART_FRAME_CMD_2DOORLOCK_SET 0x06
#define UART_FRAME_CMD_2DOORLOCK_GET_STATUS 0x07
#define UART_FRAME_CMD_LIGHT_SET 0x08
#define UART_FRAME_CMD_LIGHT_GET_STATUS 0x09
#define UART_FINGER_CMD_POWER_SET 0x0a
#define UART_FINGER_CMD_GET_STATUS  0x0b

#define UART_FRAME_CMD_ACK  0x01

#define UART_FRAME_REPORT_DOORLOCK		0x84
#define UART_FRAME_REPORT_HIGHDET		0x82
#define UART_FRAME_REPORT_SLIDEDOOR	0x81
#define UART_FRAME_REPORT_TEMP  0x83
/*
UPLOAD INFO
*/
int uart_protocal_send(uint8_t cmd, uint32_t len, uint8_t *param);




#endif

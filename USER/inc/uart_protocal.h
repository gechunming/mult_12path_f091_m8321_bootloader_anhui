#ifndef __UART_PROTOCAL_H
#define __UART_PROTOCAL_H

typedef __packed struct _UartFramePack {
	uint8_t sFlag;
	uint8_t cmd;
	uint8_t len;
	//uint8_t buf[0];			//ignore last check byte
} UartFramePack;

#define UART_FRAME_PACK_START  0xFA
#define UART_FRAME_CMD_ACKREPORT 0x00
#define UART_FRAME_CMD_REPORTKEY 0x01
#define UART_FRAME_CMD_LEDSET 0x02
#define UART_FRAME_CMD_PWMSET  0x03

int uart_protocal_send(uint8_t cmd, uint32_t len, uint8_t *param);
#endif

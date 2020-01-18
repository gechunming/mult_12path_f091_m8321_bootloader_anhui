#ifndef __TCP_PROTOCAL_H
#define __TCP_PROTOCAL_H

typedef __packed struct _TCPFramePack {
	uint8_t len;
	//uint8_t buf[0];			//ignore last check byte
} TCPFramePack;

#define TCP_FRAME_PACK_START  0xFA
#define TCP_FRAME_CMD_ACKREPORT 0x00
#define TCP_FRAME_CMD_REPORTKEY 0x01
#define TCP_FRAME_CMD_LEDSET 0x02
#define TCP_FRAME_CMD_PWMSET  0x03
#define TCP_FRAME_CMD_GPS  0x04

int TCP_protocal_send(uint32_t len, uint8_t *param);
uint32_t handle_TCP_recv_data(uint8_t *pbuf, uint32_t len);
#endif

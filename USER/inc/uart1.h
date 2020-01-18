#ifndef __UART_1_H
#define __UART_1_H


#define UART_FRAME_PACK_START  0x7E
#define UART_FRAME_PACK_TX_START 0x55
#define UART_FRAME_PACK_RX_START 0xAA
#define UART_FRAME_PACK_END 0x0D

#define IOBOARD_COMMAND_RELAY_SET  	0x01			 //index, val
#define IOBOARD_COMMAND_RELAY_GET	  0x02 			 //index
#define IOBOARD_COMMAND_INPUT_MODULE_GET 0x03  //index




#define IOBOARD_REPORT_RELAY  0x01   					//index val
#define IOBOARD_REPORT_INPUT_MODULE  0x02			//index val



#endif



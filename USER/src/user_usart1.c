#include "sx1278.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#include "contiki.h"
/*
used for at command
*/
PROCESS(process_rfid, "rfid process");

#define UART1_DMA_BUF 32
uint8_t g_uart1_dmabuf[UART1_DMA_BUF];

#define MAX_UART_BUFFER_SIZE 64
uint8_t g_uart1_buf[MAX_UART_BUFFER_SIZE];
volatile uint32_t g_uart1_index = 0;

enum {
	PROCESS_ACTION_RFID_CALLED = 1,
};
void USART1_dma_config()
{
	DMA_InitTypeDef DMA_InitStructure;
		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//DMA1??5??
DMA_DeInit(DMA1_Channel3);
//????
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
//????
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_uart1_dmabuf;
//dma??????
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//??DMA??????????
DMA_InitStructure.DMA_BufferSize = UART1_DMA_BUF;
//??DMA???????,????
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//??DMA???????
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//??????
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//??????
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//??DMA?????
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//??DMA?????
DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//??DMA?2?memory????????
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
DMA_Init(DMA1_Channel3,&DMA_InitStructure);
//????5
DMA_Cmd(DMA1_Channel3,ENABLE);
}

void USART1_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configure clock GPIO, USARTs */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);;//Reset USART1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
  /* USART1 Pins configuration  ***********************************************/  
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);   
	
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	//init_doorlock_gpio();
	USART1_dma_config();
	/* 8xUSARTs configuration --------------------------------------------------*/
  /* 8xUSARTs  configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
	USART_InitStructure.USART_BaudRate = 115200;//115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART1->ICR |= 1<<4; //?????IDLE??,??????IDLE??
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

	/*USART1 IT ENABLE*/
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	
	/*ENABLE USART1*/
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1,ENABLE);	
	
	/* USART1 IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static uint32_t handle_uart_recv_data(uint8_t *buf, uint32_t len)
{
	uint32_t rfidid;
	if (len < 11) { return 0;}
	if ((buf[0] != 0xAB) || (buf[11] != 0xA5)) return 0;
	
	rfidid = ((uint32_t)buf[5] << 24)| ((uint32_t)buf[6] << 16)| ((uint32_t)buf[7] << 8) | (uint32_t)buf[8];
	
	process_post(&process_rfid, PROCESS_ACTION_RFID_CALLED, (process_data_t)rfidid);
	return 0;
}
  

void USART1_IRQHandler(void)
{
	uint16_t Len = 0;

	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART1->ICR |= 1<<4; //????
		DMA_Cmd(DMA1_Channel3,DISABLE);
		Len = UART1_DMA_BUF - DMA_GetCurrDataCounter(DMA1_Channel3);
		if (Len < MAX_UART_BUFFER_SIZE) {
			memcpy(g_uart1_buf, g_uart1_dmabuf, Len);
			g_uart1_index = Len;
		} else {
			memcpy(g_uart1_buf, g_uart1_dmabuf, MAX_UART_BUFFER_SIZE);
			g_uart1_index = MAX_UART_BUFFER_SIZE;
		}
	
		DMA_SetCurrDataCounter(DMA1_Channel3,UART1_DMA_BUF);
		//??DMA
		DMA_Cmd(DMA1_Channel3,ENABLE);
		g_uart1_index = handle_uart_recv_data(g_uart1_buf, g_uart1_index);
	}
	USART_ClearITPendingBit(USART1, USART_IT_ORE);
}


void app_uart_puts(uint8_t *buf, uint32_t len)
{
	uint32_t i = 0;
	for (i = 0; i < len; i++) {
		USART_SendData(USART1, buf[i]);
	}
}

void print_string(char *str)
{
	while(*str) {
		USART_SendData(USART1, *str);
		str++;
	}
	USART_SendData(USART1,'\r');
	USART_SendData(USART1,'\n');
}

void print_rssi(char rssi)
{
	char buf[16];
	sprintf(buf, "rssi=%d", rssi);
	print_string(buf);
}

void print_modem_state(uint8_t state)
{
	char buf[16];
	sprintf(buf, "state=%02x", state);
	print_string(buf);
}
void print_stat_stat(uint8_t state, uint8_t state2)
{
	char buf[16];
	sprintf(buf, "stat=%02x,%02x", state, state2);
	print_string(buf);
}

void print_tx_send_cur_state(uint8_t state)
{
	char buf[16];
	sprintf(buf, "tx1 send=%02x", state);
	print_string(buf);
}

void print_addr(uint8_t *str, void *p)
{
	char buf[32];
	sprintf(buf, "%s,%08x", str, p);
	print_string(buf);
}

void print_int(uint8_t *str, int val)
{
	char buf[32];
	sprintf(buf, "%s,%d", str, val);
	print_string(buf);	
}
void print_int2(uint8_t *str, int val, int val2)
{
	char buf[32];
	sprintf(buf, "%s,%d,%d", str, val, val2);
	print_string(buf);	
}

void report_iccard(uint32_t iccard);
//volatile uint32_t mRfidCardId = 0x00;
PROCESS_THREAD(process_rfid, ev, data)
{

  PROCESS_BEGIN();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				break;
			
			case PROCESS_ACTION_RFID_CALLED:
				//mRfidCardId = (uint32_t)data;
				report_iccard((uint32_t)data);
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}



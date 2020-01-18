#include "user_USART2.h"
#include "stdbool.h"
#include "string.h"
#include "app_fifo.h"
#include <stdio.h>
#include "contiki.h"

#define MAX_FRAME_LEN   40
typedef __packed struct _UartFramePack {
	uint8_t sFlag;
	uint8_t sTxRxFlag;
	uint8_t cmd;
	uint8_t len;
	//uint8_t buf[0];			//ignore last check byte
} UartFramePack;

#define MAX_UART_BUFFER_SIZE 128
uint8_t g_uart1_buf[MAX_UART_BUFFER_SIZE];
volatile uint32_t g_uart1_index = 0;

#define UART1_DMA_BUF 64
uint8_t g_uart1_dmabuf[UART1_DMA_BUF];

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

void USART1_Init()
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


static uint32_t handle_uart_recv_data(uint8_t *pbuf, uint32_t len);
extern uint32_t getCurTicks(void);
static uint32_t gLastRecvTime = 0x00;
#if 0
void USART1_IRQHandler(void)                	//
{
	
	uint16_t recvdata = 0x00;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //
	{

		recvdata = USART_ReceiveData(USART1);//(USART1->DR);	//
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
		if (getCurTicks() - gLastRecvTime > 500) {
			g_uart1_index = 0;
		}
		g_uart1_buf[g_uart1_index++] = (uint8_t)recvdata;
		g_uart1_index = handle_uart_recv_data(g_uart1_buf, g_uart1_index);
		
		gLastRecvTime = getCurTicks();
	}
		
	USART_ClearITPendingBit(USART1, USART_IT_ORE);
} 
#endif
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

void Serial_PutByte(uint8_t data)
{
	USART_SendData(USART1, data);
}


/*
 uart tranfer protocal
 
 | byte 1 | byte 2 | byte 3 | byte 4 | ... | byte N | last     |
 | FD			| cmd    | len    |    Data 						  | checksum |
 */
PROCESS(process_uart_protocal, "uart protocal process");
//static struct etimer gSendTimer;
enum {
	EV_UART_SEND_CALLED,
	EV_UART_RECV_CALLED,
};

typedef struct
{
    UartFramePack **   p_pack_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} UartPackFifo;

static uint32_t uart_pack_length(UartPackFifo * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

volatile void *pDebugAddr = NULL;
/**@brief Put one byte to the FIFO. */
static void uart_pack_put(UartPackFifo * p_fifo, UartFramePack *pPack)
{
		pDebugAddr = (void*)pPack;
    p_fifo->p_pack_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = pPack;
    p_fifo->write_pos++;
}

/**@brief Look at one byte in the FIFO. */
static void uart_pack_peek(UartPackFifo * p_fifo, uint16_t index, UartFramePack **pPack)
{
    *pPack = p_fifo->p_pack_buf[(p_fifo->read_pos + index) & p_fifo->buf_size_mask];
}

/**@brief Get one byte from the FIFO. */
static void uart_pack_get(UartPackFifo * p_fifo, UartFramePack **pPack)
{
    uart_pack_peek(p_fifo, 0, pPack);
    p_fifo->read_pos++;
}

#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

static int32_t uart_pack_fifo_init(UartPackFifo * p_fifo, UartFramePack **p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL)
    {
        return -1;
    }

    // Check that the buffer size is a power of two.
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return -1;
    }

    p_fifo->p_pack_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return 0;
}

/*
* here for uart pack fifo buff
*/
#define MAX_UART_FRAME_NUMBER 8
static UartFramePack *gUartRecvFramePack[MAX_UART_FRAME_NUMBER];
static UartPackFifo gUartRecvPackFifo;

static UartFramePack *gUartSendFramePack[MAX_UART_FRAME_NUMBER];
static UartPackFifo gUartSendPackFifo;

int uart_protocal_init(void) 
{
	int ret;
	
	ret = uart_pack_fifo_init(&gUartRecvPackFifo, gUartRecvFramePack, MAX_UART_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	
	ret = uart_pack_fifo_init(&gUartSendPackFifo, gUartSendFramePack, MAX_UART_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	return 0;
}	
	

static void sendUartPackFrameDirect(UartFramePack *pFrame)
{
	uint8_t *pbuf = (uint8_t *)pFrame;
	uint32_t len = (pFrame->len + sizeof(UartFramePack) + 1);
	uint32_t i;

	for (i = 0; i < len; i++) {
		Serial_PutByte(pbuf[i]);
	}
}

static UartFramePack *dupPackFrame(UartFramePack *pFrame)
{
	uint32_t len = (pFrame->len) + sizeof(UartFramePack) + 1;
	uint8_t *pbuf = (uint8_t*)malloc(len);
	
	memcpy((void*)pbuf, (void*)pFrame, len);
	
	return (UartFramePack *)pbuf;
}

static void sendUartPackFrame(UartFramePack *pFrame)
{
	if (uart_pack_length(&gUartSendPackFifo) > (MAX_UART_FRAME_NUMBER/2)) {
		return;
	}
	uart_pack_put(&gUartSendPackFifo, dupPackFrame(pFrame));
	process_post(&process_uart_protocal, EV_UART_SEND_CALLED, NULL);
}

static bool checksum_sucess(UartFramePack *pFrame)
{
	uint8_t *pdata = ((uint8_t*)pFrame + sizeof(pFrame));
	uint32_t i;
	uint8_t checksum = 0;
	
	for (i = 0; i < (pFrame->len -1); i++) {
			checksum += pdata[i];
	}
	if ((checksum == pdata[i]) && (pdata[i+1] == UART_FRAME_PACK_END)) {
		return true;
	} else {
		return false;
	}
}

static uint8_t checksum_value(UartFramePack *pFrame)
{
	uint8_t *pdata = (uint8_t*)pFrame + sizeof(UartFramePack);
	uint32_t i;
	uint8_t checksum = 0;
	
	for (i = 0; i < (pFrame->len -1); i++) {
		checksum += pdata[i];
	}

	return checksum;
}


//void print_string(char *str);
static uint32_t handle_uart_recv_data(uint8_t *pbuf, uint32_t len)
{
	uint8_t *pstart = pbuf;
	uint8_t *pend = pbuf + len;
	UartFramePack *pFrame = NULL;
	
	if (len > (MAX_UART_BUFFER_SIZE-2)) return 0; //throw rubbish data
	if (len < sizeof(UartFramePack)) return len;
	
	while((*pbuf!=UART_FRAME_PACK_START) && (pbuf < pend)) {
		pbuf++;
	}
	len = pend - pbuf;
	if (pbuf != pstart)	{
		if (len > 0) {
			memcpy((void*)pstart, (void*)pbuf, len);
			pstart = pbuf;
		} else {
			return 0;
		}
	}
	
	if (len < (sizeof(UartFramePack))) { 
		return len;
	}
	
	//here got one frame start
	pFrame = (UartFramePack*)pstart;
	if ((pFrame->len < 1) || (pFrame->len > MAX_FRAME_LEN)) {
		//check frame failed then return
		return 0;
	}
	if ((sizeof(UartFramePack) + pFrame->len + 1) > len) { //not enought data
		if (pstart != pbuf) {
			memcpy((void*)pstart, (void*)pbuf, len);
		}
		return len;
	}
	
	//here got one frame, then check sum
//	print_string("got one package\n");
	if (checksum_sucess(pFrame)) {
		//here use for process this pFrame
	//	printf("checksum ok\n");
		//print_string("got frame\n");
		if (uart_pack_length(&gUartRecvPackFifo) < 5) {
			uart_pack_put(&gUartRecvPackFifo, dupPackFrame(pFrame));
			process_post(&process_uart_protocal, EV_UART_RECV_CALLED, NULL);
		}
	} else {
		//print_string("check failed\n");
	}
	/*
	pbuf += (pFrame->len + sizeof(UartFramePack) + 1);

	if (pend != pbuf) {
		memcpy((void*)pstart, (void*)pbuf, (pend-pbuf));
		return pend-pbuf;
	} else {
		return 0;
	}
	*/
	return 0;
}

int  SendCmdAck(uint8_t cmd, uint16_t value) 
{
//	int i;
	uint8_t sbuf[16];
	UartFramePack *pSendFrame = (UartFramePack *)sbuf;
	uint8_t *pParam = sbuf + sizeof(UartFramePack);
	pSendFrame->sFlag = UART_FRAME_PACK_START;
	pSendFrame->sTxRxFlag = UART_FRAME_PACK_TX_START;
	pSendFrame->cmd = UART_FRAME_CMD_ACK;
	pSendFrame->len = 0x05;
	pParam[0] = 0x55;
	pParam[1] = cmd;
	pParam[2] = value >> 8;
	pParam[3] = value & 0xff;
	
	pParam[4] = checksum_value(pSendFrame);
	pParam[5] = 0x0d;
	
	sendUartPackFrame(pSendFrame);
	return 0;
}

int uart_protocal_send(uint8_t cmd, uint32_t len, uint8_t *param)
{
	int i;
	uint8_t sbuf[64];
	UartFramePack *pSendFrame = (UartFramePack *)sbuf;
	uint8_t *pParam = sbuf + sizeof(UartFramePack);
	pSendFrame->sFlag = UART_FRAME_PACK_START;
	pSendFrame->sTxRxFlag = UART_FRAME_PACK_TX_START;
	pSendFrame->cmd = cmd;
	pSendFrame->len = len + 1;
	
	for (i = 0; i < len; i++) {
		pParam[i] = param[i];
	}
	
	pParam[len] = checksum_value(pSendFrame);
	pParam[len + 1] = 0x0d;
	
	sendUartPackFrame(pSendFrame);
	return 0;
}

void send_doorlock_status(uint8_t status)
{
	uart_protocal_send(UART_FRAME_REPORT_DOORLOCK, 1, &status);
} 

void send_highdet_status(uint8_t status)
{
	uart_protocal_send(UART_FRAME_REPORT_HIGHDET, 1, &status);
}
void send_slidedoor_status(uint8_t status) 
{
	uart_protocal_send(UART_FRAME_REPORT_SLIDEDOOR, 1, &status);
}

void send_temp_report(uint16_t status) 
{
	uart_protocal_send(UART_FRAME_REPORT_TEMP, 2, (uint8_t*)&status);
}


void led_set(uint8_t index, uint8_t val);
void door_open(void);
void user_call_slide_action(bool open);
void user_call_light_action(bool open);
void user_call_fan_action(bool open);
void user_call_finger_power(bool open);
uint16_t user_get_current_slide_door_status(void);
uint16_t user_get_high_det_status(void);
uint16_t user_get_temp(void);
uint16_t user_get_door_lock_status(void);
uint16_t user_call_get_light_status(void);
static void handle_uart_protocal_receive(void)
{
	UartFramePack *pFrame = NULL;
	uint8_t *pData = NULL;

	while (uart_pack_length(&gUartRecvPackFifo) > 0) {
		uart_pack_get(&gUartRecvPackFifo, &pFrame);
		pData = ((uint8_t *)pFrame) + sizeof(UartFramePack);
		switch(pFrame->cmd) {
		
			case UART_FRAME_CMD_SLIDEDOOR_OPEN_GET_STATUS:
				SendCmdAck(UART_FRAME_CMD_SLIDEDOOR_OPEN_GET_STATUS, user_get_current_slide_door_status());
				break;
			
			case UART_FRAME_CMD_SLIDEDOOR_SET:
				user_call_slide_action(pData[0] != 0x00);
				SendCmdAck(UART_FRAME_CMD_SLIDEDOOR_SET, 0x00);
				break;
			
			case UART_FRAME_CMD_HIGH_GET_STATUS:
				SendCmdAck(UART_FRAME_CMD_HIGH_GET_STATUS, user_get_high_det_status());
				break;
			
			case UART_FRAME_CMD_GET_TEMP:
				SendCmdAck(UART_FRAME_CMD_GET_TEMP, user_get_temp());
				break;
			
			case UART_FRAME_CMD_2DOORLOCK_SET:
				SendCmdAck(UART_FRAME_CMD_2DOORLOCK_SET, 0x00);
				door_open();
				break;
			
			case UART_FRAME_CMD_2DOORLOCK_GET_STATUS:
				SendCmdAck(UART_FRAME_CMD_2DOORLOCK_GET_STATUS, user_get_door_lock_status());
				break;
			
			case UART_FRAME_CMD_LIGHT_SET:
				user_call_light_action(pData[0] != 0x00);
				SendCmdAck(UART_FRAME_CMD_LIGHT_SET, 0x00);
				break;
			
			case UART_FRAME_CMD_LIGHT_GET_STATUS:
				SendCmdAck(UART_FRAME_CMD_LIGHT_GET_STATUS, user_call_get_light_status());
				break;
			
			case UART_FINGER_CMD_POWER_SET:
				user_call_finger_power(pData[0] != 0x00);
			SendCmdAck(UART_FINGER_CMD_POWER_SET, pData[0] != 0x00);
				break;
			
			default:
				break;
		}
		free((void*)pFrame);
	}
}


static void handle_uart_protocal_send(void)
{
	UartFramePack *pFrame = NULL;
//	uint8_t *pData = NULL;
//	uint32_t cardid;
	
	while (uart_pack_length(&gUartSendPackFifo) > 0) {
		uart_pack_get(&gUartSendPackFifo, &pFrame);
		sendUartPackFrameDirect(pFrame);
		free((void*)pFrame);
	}
}

PROCESS_THREAD(process_uart_protocal, ev, data)
{

  PROCESS_BEGIN();
	
	USART1_Init();
	uart_protocal_init();

	//uart_protocal_send(0x22, 1, "ab");
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case EV_UART_SEND_CALLED:
				handle_uart_protocal_send();
				break;
				
			case EV_UART_RECV_CALLED:
				handle_uart_protocal_receive();
				break;
			
			default:
				break;
		}
  }
  PROCESS_END();
}


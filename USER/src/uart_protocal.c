#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "app_fifo.h"
#include "led.h"
#include "uart_protocal.h"
#include "userpwm.h"

/*
 uart tranfer protocal
 
 | byte 1 | byte 2 | byte 3 | byte 4 | ... | byte N | last     |
 | FD			| cmd    | len    |    Data 						  | checksum |
 */
PROCESS(process_uart_protocal, "uart protocal process");
static struct etimer gUartSendRetryEtimer;
enum {
	EV_UART_SEND_CALLED,
	EV_UART_RECV_CALLED,
};

enum UartProtocalStatus {
	UART_STATUS_IDLE,
	UART_SEND_WAITACK,
};
typedef struct
{
    UartFramePack **          p_pack_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} UartPackFifo;


static uint32_t uart_pack_length(UartPackFifo * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

/**@brief Put one byte to the FIFO. */
static void uart_pack_put(UartPackFifo * p_fifo, UartFramePack *pPack)
{
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
static UartFramePack *gUartFramePack[MAX_UART_FRAME_NUMBER];
static UartPackFifo gUartPackFifo;
static UartFramePack *gUartRecvFramePack[MAX_UART_FRAME_NUMBER];
static UartPackFifo gUartRecvPackFifo;

static enum UartProtocalStatus gUartProtocalStatus = UART_STATUS_IDLE;
static UartFramePack *gUartCurSendFrame = NULL;
#define MAX_RETRY_SEND_TIME 3
static uint8_t gSendRetryTimes = 0;
int uart_protocal_init(void) 
{
	int ret;
	
	ret = uart_pack_fifo_init(&gUartPackFifo, gUartFramePack, MAX_UART_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	
	ret = uart_pack_fifo_init(&gUartRecvPackFifo, gUartRecvFramePack, MAX_UART_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	return 0;
}	
	
/*
for uart 2
*/
#define MAX_UART_BUFFER_SIZE  128
static uint8_t g_uart2_buf[MAX_UART_BUFFER_SIZE];
static uint32_t g_uart2_index = 0; //next receive byte
void USART2_Init()
{

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configure clock GPIO, USARTs */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//Reset USART2
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
  /* USART2 Pins configuration  ***********************************************/  
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);   
	
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

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
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
	
	/*USART2 IT ENABLE*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  /* USART2 IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/*ENABLE USART2*/
	USART_Cmd(USART2,ENABLE);	
}


/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
static uint32_t handle_uart_recv_data(uint8_t *pbuf, uint32_t len);
void USART2_IRQHandler(void)
{
	uint16_t recvdata = 0x00;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //
	{
		recvdata = USART_ReceiveData(USART2);//(USART1->DR);	//
		g_uart2_buf[g_uart2_index++] = (uint8_t)recvdata;
		g_uart2_index = handle_uart_recv_data(g_uart2_buf, g_uart2_index);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

void sendUartPackFrame(UartFramePack *pFrame)
{
	uint8_t *pbuf = (uint8_t *)pFrame;
	uint32_t len = (pFrame->len + sizeof(UartFramePack) + 1);
	uint32_t i;

	for (i = 0; i < len; i++) {
		USART_SendData(USART2, pbuf[i]);
	}
}


static void uart_send_ack(uint8_t cmd)
{
	uint32_t i;
	uint8_t ackbuf[5];
	uint8_t checksum = 0;
	ackbuf[0] = UART_FRAME_PACK_START;
	ackbuf[1] = UART_FRAME_CMD_ACKREPORT;
	ackbuf[2] = 0x01;
	ackbuf[3] = cmd;
	
	for (i = 0; i < 4; i++) {
		checksum += ackbuf[i];
	}
	ackbuf[i] = checksum;
	
	for (i = 0; i < 5; i++) {
		USART_SendData(USART2, ackbuf[i]);
	}
}

static bool checksum_sucess(UartFramePack *pFrame)
{
	uint8_t *pbuf = (uint8_t*)pFrame;
	uint32_t i;
	uint8_t checksum = 0;
	
	for (i = 0; i < (pFrame->len + sizeof(UartFramePack)); i++) {
			checksum += pbuf[i];
	}
	if (checksum == pbuf[i]) {
		return true;
	} else {
		return false;
	}
}

static UartFramePack *dupPackFrame(UartFramePack *pFrame)
{
	uint32_t len = (pFrame->len) + sizeof(UartFramePack) + 1;
	uint8_t *pbuf = (uint8_t*)malloc(len);
	
	memcpy((void*)pbuf, (void*)pFrame, len);
	
	return (UartFramePack *)pbuf;
}
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
	
	if (len < (sizeof(UartFramePack))) { 
		if (pstart != pbuf) {
			memcpy((void*)pstart, (void*)pbuf, len);
		}
		return len;
	}
	
	//here got one frame start
	pFrame = (UartFramePack*)pbuf;
	if ((sizeof(UartFramePack) + pFrame->len + 1) > len) { //not enought data
		if (pstart != pbuf) {
			memcpy((void*)pstart, (void*)pbuf, len);
		}
		return len;
	}
	
	//here got one frame, then check sum
	if (checksum_sucess(pFrame)) {
		//here use for process this pFrame
		uart_pack_put(&gUartRecvPackFifo, dupPackFrame(pFrame));
		process_post(&process_uart_protocal, EV_UART_RECV_CALLED, NULL);
	}
	pbuf += (pFrame->len + sizeof(UartFramePack) + 1);
	if (pend != pbuf) {
		memcpy((void*)pstart, (void*)pbuf, (pend-pbuf));
		return pend-pbuf;
	} else {
		return 0;
	}
}

int uart_protocal_send(uint8_t cmd, uint32_t len, uint8_t *param)
{
	UartFramePack *pFrame = NULL;
	uint8_t *pParam = NULL, *pbuf;
	int i;
	uint8_t checksum = 0;
	
	if (uart_pack_length(&gUartPackFifo) >= MAX_UART_FRAME_NUMBER) {
		return -1; //for buf is full
	}
	pFrame = (UartFramePack*) malloc(len + sizeof(UartFramePack) + 1);
	
	pFrame->cmd = cmd;
	pFrame->len = len;
	pFrame->sFlag = UART_FRAME_PACK_START;
	
	pParam = ((uint8_t*)pFrame) + sizeof(UartFramePack);
	memcpy((void*)pParam, (const void *)param, len);
	
	pbuf = (uint8_t*)pFrame;
	for (i = 0; i < (len + sizeof(UartFramePack)); i++) {
		checksum += pbuf[i];
	}
	pbuf[i] = checksum;
	
	uart_pack_put(&gUartPackFifo, pFrame);
	process_post(&process_uart_protocal, EV_UART_SEND_CALLED, NULL);
	
	return 0;
}


static void send_next_uart_protocal_command(void)
{
	if(uart_pack_length(&gUartPackFifo) > 0) {
		uart_pack_get(&gUartPackFifo, &gUartCurSendFrame);
		sendUartPackFrame(gUartCurSendFrame);
		gUartProtocalStatus = UART_SEND_WAITACK;
		gSendRetryTimes = 1;
		etimer_set(&gUartSendRetryEtimer, ((unsigned long)100 * CLOCK_SECOND)/ 1000);
	}	else {
		gUartProtocalStatus = UART_STATUS_IDLE;
	}		
}
static void handle_uart_protocal_receive(void)
{
	UartFramePack *pFrame = NULL;
	uint8_t *pParam = NULL;
	while (uart_pack_length(&gUartRecvPackFifo) > 0) {
		uart_pack_get(&gUartRecvPackFifo, &pFrame);
		pParam = ((uint8_t *)pFrame) + sizeof(UartFramePack);
		switch(pFrame->cmd) {
			case UART_FRAME_CMD_ACKREPORT:
				if (gUartProtocalStatus == UART_SEND_WAITACK) {
					if (gUartCurSendFrame != NULL) 
						free((void*)gUartCurSendFrame);
					gUartCurSendFrame = NULL;
					send_next_uart_protocal_command();
				}
				break;
				
			case UART_FRAME_CMD_LEDSET:
				if (pFrame->len == 2) {
					led_set(pParam[0], pParam[1]);
				}
				uart_send_ack(UART_FRAME_CMD_LEDSET);
				break;
				
			case UART_FRAME_CMD_PWMSET:
				if (pFrame->len == 2) {
					uint8_t percent = pParam[1];
					if (percent > 100) percent = 100;
					change_pwm_ouput_percent(pParam[0], percent);
				}
				break;
				
			default:
				break;
		}
		free((void*)pFrame);
	}
}


PROCESS_THREAD(process_uart_protocal, ev, data)
{

  PROCESS_BEGIN();
	USART2_Init();
	uart_protocal_init();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case EV_UART_SEND_CALLED:
				if (gUartProtocalStatus == UART_STATUS_IDLE) {	
					send_next_uart_protocal_command();
				}
				break;
				
			case EV_UART_RECV_CALLED:
				handle_uart_protocal_receive();
				break;
				
			case PROCESS_EVENT_TIMER:
				if (gUartProtocalStatus == UART_SEND_WAITACK) {	//timeout
					if (gSendRetryTimes++ < MAX_RETRY_SEND_TIME) {
						sendUartPackFrame(gUartCurSendFrame);
						etimer_set(&gUartSendRetryEtimer, ((unsigned long)100 * CLOCK_SECOND)/ 1000);
					} else {
						if (gUartCurSendFrame != NULL) 
							free((void*)gUartCurSendFrame);
						gUartCurSendFrame = NULL;
						send_next_uart_protocal_command();					
					}
				}
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


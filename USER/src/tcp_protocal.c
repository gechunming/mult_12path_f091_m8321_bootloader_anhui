#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "app_fifo.h"
#include "led.h"
#include "tcp_protocal.h"
#include "user_usart1.h"
#include "userpwm.h"
#include "saferopemachine.h"
/*
 TCP tranfer protocal.
 It's seem to uart protocal.
 
 | byte 1 | byte 2 | byte 3 | byte 4 | ... | byte N | last     |
 | FD			| cmd    | len    |    Data 						  | checksum |
 
  FA 02 02 00 01 ff   light led 1  
	FA 02 02 00 00 FE		off led 1
	
 */
PROCESS(process_TCP_protocal, "TCP protocal process");
static struct etimer gTCPSendRetryEtimer;
#define TCP_SEND_WAIT_SENDED 10
#define TCP_SEND_WAIT_ACK 30  //5 seconds
enum {
	EV_TCP_SEND_CALLED,
	EV_TCP_RECV_CALLED,
};

enum TCPProtocalStatus {
	TCP_STATUS_IDLE,
	TCP_STATUS_WAITSENT,
	TCP_SEND_WAITACK,
};
typedef struct
{
    TCPFramePack **          p_pack_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} TCPPackFifo;

/*
* here for TCP pack fifo buff
* all command frame will packed to TcpFramePack
* all command frame which received by tcp, will put to TcpRcvFramePack

* for Rx
* at process --> gTCPRecvPackFifo  --> tcp processor
*  
* for Tx
* other process --> gTcpPackFifo  --> tcp processor --> put to at buffer --> at processor
*
*/
#define MAX_TCP_FRAME_NUMBER 8
static TCPFramePack *gTCPFramePack[MAX_TCP_FRAME_NUMBER];
static TCPPackFifo gTCPPackFifo;

static TCPFramePack *gTCPRecvFramePack[MAX_TCP_FRAME_NUMBER];
static TCPPackFifo gTCPRecvPackFifo;

/*
current sending command
current retry times
every command will retry 3 times.
*/
static enum TCPProtocalStatus gTCPProtocalStatus = TCP_STATUS_IDLE;
static TCPFramePack *gTCPCurSendFrame = NULL;
#define MAX_RETRY_SEND_TIME 3
static uint8_t gSendRetryTimes = 0;

static uint32_t TCP_pack_length(TCPPackFifo * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

/**@brief Put one byte to the FIFO. */
static void TCP_pack_put(TCPPackFifo * p_fifo, TCPFramePack *pPack)
{
    p_fifo->p_pack_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = pPack;
    p_fifo->write_pos++;
}

/**@brief Look at one byte in the FIFO. */
static void TCP_pack_peek(TCPPackFifo * p_fifo, uint16_t index, TCPFramePack **pPack)
{
    *pPack = p_fifo->p_pack_buf[(p_fifo->read_pos + index) & p_fifo->buf_size_mask];
}

/**@brief Get one byte from the FIFO. */
static void TCP_pack_get(TCPPackFifo * p_fifo, TCPFramePack **pPack)
{
    TCP_pack_peek(p_fifo, 0, pPack);
    p_fifo->read_pos++;
}

#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

static int32_t TCP_pack_fifo_init(TCPPackFifo * p_fifo, TCPFramePack **p_buf, uint16_t buf_size)
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



static int TCP_protocal_init(void) 
{
	int ret;
	
	ret = TCP_pack_fifo_init(&gTCPPackFifo, gTCPFramePack, MAX_TCP_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	
	ret = TCP_pack_fifo_init(&gTCPRecvPackFifo, gTCPRecvFramePack, MAX_TCP_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	return 0;
}	
	
/*
for TCP 2
*/

extern struct process process_at_protocal;
void SleepAfterMs(uint32_t ms);
static void sendTCPPackFrame(TCPFramePack *pFrame)
{
	//will call tcp send directly
	process_post(&process_at_protocal, MODEM_EVENT_SEND, (process_data_t)pFrame);
	SleepAfterMs(1000 * 100);
}


/*
  parse recv data package
  +NNMI:10,37383639353433323161
*/
static uint8_t Hex2UChar(uint8_t *data)
{
	uint8_t result = 0;
	
	if ((*data >= '0') && (*data <= '9')) {
		result = (*data - '0') << 4;
	} else if ((*data >= 'A') && (*data <= 'F')) {
		result = ((*data - 'A') + 10) << 4;
	} else if ((*data >= 'a') && (*data <= 'f')) {
		result = ((*data - 'a') + 10) << 4;
	} 
	
	data++;
	
	if ((*data >= '0') && (*data <= '9')) {
		result += (*data - '0');
	} else if ((*data >= 'A') && (*data <= 'F')) {
		result += ((*data - 'A') + 10);
	} else if ((*data >= 'a') && (*data <= 'f')) {
		result += ((*data - 'a') + 10);
	} 
	
	return result;
}

void handle_tcp_recv(uint8_t *line)
{
	char rawbuf[32];
	int datalen, i;
	TCPFramePack *pFrame;
	uint8_t *pParam = NULL;
	int slen = strlen((char *)line);
	
	if (slen < 9) {
		return;
	}
	
	datalen = atoi((const char *)&line[6]);
	if (datalen * 2 != (slen-9)) {
		return;
	}
	
	for(i = 0; i < datalen; i++) {
		rawbuf[i] = Hex2UChar(&line[9+i * 2]);
	}
	
	pFrame = (TCPFramePack *)malloc(sizeof(TCPFramePack) + datalen-1);
	if (pFrame == NULL) return;
	
	pFrame->cmd = rawbuf[0];
	pFrame->len = datalen-1;
	pParam = (uint8_t*)pFrame + sizeof(TCPFramePack);
	memcpy((void*)pParam, (void*)&rawbuf[1], datalen-1);
	
	TCP_pack_put(&gTCPRecvPackFifo, pFrame);
	process_post(&process_TCP_protocal, EV_TCP_RECV_CALLED, NULL);
}

/*
 external api
*/
static void clear_tcp_package(void);
void SleepAfterMs(uint32_t ms);
int TCP_protocal_send(uint8_t cmd, uint32_t len, uint8_t *param, bool confirm)
{
	TCPFramePack *pFrame = NULL;
	uint8_t *pParam = NULL;
	
	SleepAfterMs(1000 * 100);
	
	if (TCP_pack_length(&gTCPPackFifo) >= MAX_TCP_FRAME_NUMBER) {
			process_post(&process_TCP_protocal, EV_TCP_SEND_CALLED, NULL);
		print_string("nb package fullfailed, this message will igore, and clear all\r\n");
		clear_tcp_package();
		gTCPProtocalStatus = TCP_STATUS_IDLE;
		return -1; //for buf is full
	}
	pFrame = (TCPFramePack*) malloc(len + sizeof(TCPFramePack));
	
	pFrame->cmd = cmd;
	pFrame->len = len;
	pFrame->confirmed = confirm;
	
	pParam = ((uint8_t*)pFrame) + sizeof(TCPFramePack);
	memcpy((void*)pParam, (const void *)param, len);
	
	TCP_pack_put(&gTCPPackFifo, pFrame);
	process_post(&process_TCP_protocal, EV_TCP_SEND_CALLED, NULL);
	
	return 0;
}


void handle_send_error(void)
{
	//nop will dummy it 
	//retry send it
	return;
}

static void clear_tcp_package(void)
{
	TCPFramePack *pTcpPackTemp = NULL;
	while(TCP_pack_length(&gTCPPackFifo) > 0) {
		TCP_pack_get(&gTCPPackFifo, &pTcpPackTemp);
		free((void*)pTcpPackTemp);
	}
	
	if (gTCPCurSendFrame != NULL) {
		free((void*)gTCPCurSendFrame);
		gTCPCurSendFrame = NULL;
	}
}

static void send_next_TCP_protocal_command(void)
{
	if(TCP_pack_length(&gTCPPackFifo) > 0) {
		if (gTCPCurSendFrame != NULL) {
			free(gTCPCurSendFrame);
		}
		TCP_pack_get(&gTCPPackFifo, &gTCPCurSendFrame);
		sendTCPPackFrame(gTCPCurSendFrame);
		gTCPProtocalStatus = TCP_STATUS_WAITSENT;
		gSendRetryTimes = 1;
		etimer_set(&gTCPSendRetryEtimer, ((unsigned long)TCP_SEND_WAIT_SENDED * CLOCK_SECOND));
	}	else {
		gTCPProtocalStatus = TCP_STATUS_IDLE;
	}		
}
void handle_tcp_sended(void)
{
		if (gTCPCurSendFrame->confirmed) {
			gTCPProtocalStatus = TCP_SEND_WAITACK;
			//gSendRetryTimes = 1;
	
			PROCESS_CONTEXT_BEGIN(&process_TCP_protocal);
			etimer_set(&gTCPSendRetryEtimer, ((unsigned long)TCP_SEND_WAIT_ACK * CLOCK_SECOND));
			PROCESS_CONTEXT_END(&process_TCP_protocal);
		} else {
			PROCESS_CONTEXT_BEGIN(&process_TCP_protocal);
			etimer_stop(&gTCPSendRetryEtimer);
			PROCESS_CONTEXT_END(&process_TCP_protocal);
			
			if (gTCPCurSendFrame != NULL) 
				free((void*)gTCPCurSendFrame);
			gTCPCurSendFrame = NULL;
			send_next_TCP_protocal_command();
		}
}

extern struct process process_safemachine;
static void handle_TCP_protocal_receive(void)
{
	TCPFramePack *pFrame = NULL;
	//uint8_t *pParam = NULL;
	while (TCP_pack_length(&gTCPRecvPackFifo) > 0) {
		TCP_pack_get(&gTCPRecvPackFifo, &pFrame);
	//	pParam = ((uint8_t *)pFrame) + sizeof(TCPFramePack);
		switch(pFrame->cmd) {
			case TCP_FRAME_CMD_ACKREPORT:
				if (gTCPProtocalStatus == TCP_SEND_WAITACK) {
					if (gTCPCurSendFrame != NULL) 
						free((void*)gTCPCurSendFrame);
					gTCPCurSendFrame = NULL;
					send_next_TCP_protocal_command();
				}
				break;
			
			case SAFEMACHINE_NB_UPLINK_LOCKSTATUS_CMD:
			case SAFEMACHINE_NB_DOWNLINK_LOCK_CMD:
				 process_post(&process_safemachine, SAFE_MACHINE_EVENT_REQUESTUNLOCK, NULL);
				break;
			
			case SAFEMACHINE_NB_DOWNLINK_REQUEST_RESETH0_CMD:
				process_post(&process_safemachine, SAFE_MACHINE_EVENT_NBREQUESTRESETH0, NULL);
				break;
			
			default:
				break;
		}
		free((void*)pFrame);
	}
}

static void handle_process_timer(void)
{
	unsigned long sendinterval = TCP_SEND_WAIT_ACK * CLOCK_SECOND;
	
	switch(gTCPProtocalStatus) {
		case TCP_STATUS_WAITSENT:
			sendinterval = TCP_SEND_WAIT_SENDED * CLOCK_SECOND;
		case TCP_SEND_WAITACK:
			if (gSendRetryTimes++ < MAX_RETRY_SEND_TIME) {
				sendTCPPackFrame(gTCPCurSendFrame);
				etimer_set(&gTCPSendRetryEtimer, sendinterval);
			} else {
				if (gTCPCurSendFrame != NULL) 
					free((void*)gTCPCurSendFrame);
				gTCPCurSendFrame = NULL;
				clear_tcp_package();
				//send_next_TCP_protocal_command();	
				//will restart nbiot for reconnect
				gTCPProtocalStatus = TCP_STATUS_IDLE;
				process_post(&process_at_protocal, MODEM_EVENT_REINIT, NULL);
			}
			break;
		
		default:
			break;
	}
}

PROCESS_THREAD(process_TCP_protocal, ev, data)
{

  PROCESS_BEGIN();
	
	TCP_protocal_init();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case EV_TCP_SEND_CALLED:
				if (gTCPProtocalStatus == TCP_STATUS_IDLE) {	
					send_next_TCP_protocal_command();
				}
				break;
				
			case EV_TCP_RECV_CALLED:

				handle_TCP_protocal_receive();
				break;
				
			case PROCESS_EVENT_TIMER:
				handle_process_timer();
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


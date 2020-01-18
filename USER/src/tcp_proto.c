#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "app_fifo.h"
#include "tcp_protocal.h"
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
#define TCP_SEND_WAIT_ACK 5  //5 seconds
enum {
	EV_TCP_SEND_CALLED,
	EV_TCP_RECV_CALLED,
	EV_TCP_SEND_OVER,
};

enum TCPProtocalStatus {
	TCP_STATUS_IDLE,
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

void print_addr(uint8_t *str, void *p);
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
int tcpprotocal_send_data(uint8_t *buf, int len);
static void sendTCPPackFrame(TCPFramePack *pFrame)
{
	uint8_t *pbuf = ((uint8_t *)pFrame)+1;
	uint32_t len = pFrame->len;
	
	tcpprotocal_send_data(pbuf, len);
}

static TCPFramePack *createPackFrame(uint8_t *pData, int len)
{
	TCPFramePack *pFrame = (TCPFramePack*)malloc(len+1);
	uint8_t *pBuf = (uint8_t*)pFrame;
	
	print_addr((uint8_t*)"malloc1", pFrame);
	if (pFrame == NULL) return NULL;
	
	pFrame->len = len;
	memcpy((void*)(pBuf+1), (void*)pData, len);
	
	return pFrame;
}

/*
used by at process 
*/
void handle_tcp_send_over()
{
	process_post(&process_TCP_protocal, EV_TCP_SEND_OVER, NULL);
}
uint32_t handle_TCP_recv_data(uint8_t *buf, uint32_t len)
{
	uint32_t pcur = 0;
	uint8_t *pstart = buf;
	uint8_t *pend = buf;
	TCPFramePack *pFrame = NULL;
	
	while(pend < (buf + len)) {
		pstart = (uint8_t *)strstr((const char *)pstart, "*HQ,");
		if (pstart == NULL) {
			return len;
		}
		pend = (uint8_t *)strchr((const char *)pstart, '#');
		if (pend == NULL) {
			//still need data
			if ((buf + len - pstart) > 0) {
				memcpy(buf, pstart, (buf + len - pstart));
			}
			return (buf+len - pstart);
		}
		if (TCP_pack_length(&gTCPRecvPackFifo) < MAX_TCP_FRAME_NUMBER) {
			pFrame = createPackFrame(pstart, (pend-pstart+1));
			if (pFrame != NULL) {
				TCP_pack_put(&gTCPRecvPackFifo, pFrame);
				process_post(&process_TCP_protocal, EV_TCP_RECV_CALLED, NULL);
			}
		} else {
			print_string("tcp recv fifo full");
		}
		pend++;
		pstart = pend;
	}
	
	return pend - buf;
}

/*
 external api
*/
volatile void *mTestMemAddr = 0x00;
static void clear_tcp_package(void)
{
	TCPFramePack *pTcpPackTemp = NULL;
	while(TCP_pack_length(&gTCPPackFifo) > 0) {
		TCP_pack_get(&gTCPPackFifo, &pTcpPackTemp);
		free((void*)pTcpPackTemp);
		print_addr("free3", pTcpPackTemp);
	}
	
	if (gTCPCurSendFrame != NULL) {
		free((void*)gTCPCurSendFrame);
		print_addr("free4", gTCPCurSendFrame);
		gTCPCurSendFrame = NULL;
	}
}

volatile void *pTestAddr = NULL;
int TCP_protocal_send(uint32_t len, uint8_t *param)
{
	TCPFramePack *pFrame = NULL;
	uint8_t *pbuf;
	
	if (TCP_pack_length(&gTCPPackFifo) >= MAX_TCP_FRAME_NUMBER) {
		process_post(&process_TCP_protocal, EV_TCP_SEND_CALLED, NULL);
		//print_string("nb package fullfailed, this message will igore, and clear all\r\n");
		clear_tcp_package();
		gTCPProtocalStatus = TCP_STATUS_IDLE;
		return -1; //for buf is full
	}
	pFrame = (TCPFramePack*) malloc(len + sizeof(TCPFramePack));
	pTestAddr = pFrame;
	print_addr((uint8_t*)"malloc2", pFrame);
	if (pFrame == NULL) {
		return 0;
	}
	pFrame->len = len;
	pbuf = (uint8_t*)pFrame + 1;
	memcpy(pbuf, param, len);
	mTestMemAddr  = (void*)pFrame;
	
	TCP_pack_put(&gTCPPackFifo, pFrame);
	process_post(&process_TCP_protocal, EV_TCP_SEND_CALLED, NULL);
	
	return 0;
}

extern uint8_t IMEI[24];
#include <stdio.h>
extern volatile uint8_t m2gCsq;
void tcp_send_htbt(void)
{
	uint8_t sbuf[64];
	snprintf((char*)sbuf, 63, "*ZB,%s,HTBT,%d,100,0#", (char*)IMEI, m2gCsq);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);
	return;
}

int tcp_send_device_info(void) 
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*ZB,%s,INFO,6.2,1.0,112233445566#", (char*)IMEI);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);
	return 0;
}

int tcp_send_query_charger_info(void) 
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*ZB,%s,S-QUERY,01,0#", (char*)IMEI);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);
	return 0;
}


int tcp_send_getip(void) 
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*HQ,%s,SERVER#", (char*)IMEI);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);
	return 0;
}

bool isLockPowerOff(void);


static void send_query_result()
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*ZB,%s,H-QUERY,%s,6.1,1.0,0#", (char*)IMEI, isLockPowerOff()?"FF":"FB");
	TCP_protocal_send(strlen((char*)sbuf), sbuf);	
}

void report_iccard(uint32_t iccard)
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*ZB,%s,CNB,01,%08x,0#", (char*)IMEI, iccard);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);	
}

static void send_next_TCP_protocal_command(void)
{
	if(TCP_pack_length(&gTCPPackFifo) > 0) {
		if (gTCPCurSendFrame != NULL) {
			free(gTCPCurSendFrame);
			print_addr("free5", gTCPCurSendFrame);
		}
		
		TCP_pack_get(&gTCPPackFifo, &gTCPCurSendFrame);
		sendTCPPackFrame(gTCPCurSendFrame);
		gTCPProtocalStatus = TCP_SEND_WAITACK;
		gSendRetryTimes = 1;
		etimer_set(&gTCPSendRetryEtimer, ((unsigned long)TCP_SEND_WAIT_ACK * CLOCK_SECOND));
	}	else {
		gTCPProtocalStatus = TCP_STATUS_IDLE;
	}		
}
extern struct process process_tcp_htbt;
#include "tcp_htbt.h"
#include "lock_report.h"
//#include "core_cm4.h"
#include "lock_pwr.h"

static void send_unlock_ack(bool success, uint32_t id)
{
	uint8_t sbuf[64];
	
	snprintf((char*)sbuf, 63, "*ZB,%s,UNLK,01,%s,%d#", (char*)IMEI, success?"OK":"ER", id);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);	
}
extern struct process process_tcp_LOCK_REPORT;
extern struct process process_tcp_htbt;
extern struct process process_lock_pwr;
extern struct process process_at_protocal;
extern bool user_call_unlock(void);

extern volatile uint8_t mRemoteServerIp[4];
extern volatile uint16_t mRemoteServerPort;
#include "gsm_test.h"

static void handle_server_ip(uint8_t * par[], int parnum)
{
	uint8_t *pend = NULL;
	uint8_t *ptemp = NULL;
	uint8_t *pnext = NULL;
	int i;
	
	if (parnum < 2) {
		goto parse_ip_failed;
	}
	ptemp = par[0];
	pend = ptemp + strlen(ptemp);
	for (i = 0; i < 4; i++) {
		if (i <= 2) {
			pnext = strchr(ptemp, '.');
			if (pnext == NULL) goto parse_ip_failed;
			*pnext = '\0';
		} else {
			pnext = pend;
		}
		mRemoteServerIp[i] = atoi(ptemp);
		
		if (i == 3) break;
		
		pnext++;
		ptemp = pnext;
		if (ptemp >= pend) goto parse_ip_failed;
	}
	mRemoteServerPort = atoi(par[1]);
	goto parse_ip_ok;
	
parse_ip_failed:
	mRemoteServerIp[0] = 47;
	mRemoteServerIp[1] = 92;
	mRemoteServerIp[2] = 71;
	mRemoteServerIp[3] = 200;
	mRemoteServerPort = 6788;
parse_ip_ok:
	process_post(&process_at_protocal, MODEM_EVENT_TCPIPGOT, NULL);
	return;
}
bool unlock_after_reboot(uint32_t curpower);
static void process_one_command(uint8_t *imei, uint8_t *cmd, uint8_t  *par[], int parnum)
{
	uint32_t id;

	if (!strncmp((char*)cmd, "UNLK", 4)) {
		if ((par[1] != NULL) && (!strcmp((const char*)par[1], "OK"))) {
			//lock ack
		//	process_post(&process_tcp_LOCK_REPORT, PROCESS_EVENT_LOCK_REPORT_RECV, NULL);
			return;
		} else {
			id = atoi((const char *)par[1]);
			if (user_call_unlock()) {
				send_unlock_ack(true, id);
			} else {
				send_unlock_ack(false, id);
			}
		}
	} else if (!strncmp((char*)cmd, "H-QUERY", 7)) {
		send_query_result();
	} else if (!strncmp((char*)cmd, "LOCK", 4)) {
		if ((par[1] != NULL) && (!strcmp((const char*)par[1], "OK"))) {
			//lock ack
			process_post(&process_tcp_LOCK_REPORT, PROCESS_EVENT_LOCK_REPORT_RECV, NULL);
			return;
		} else {
			process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_DISCHARGE_COMMAND, NULL);
		}
		return;
	} else if (!strncmp((char*)cmd, "HTBT", 4)) {
			process_post(&process_tcp_htbt, PROCESS_EVENT_TCP_HTBT_RECV, NULL);
			return;
	} else if (!strncmp((char*)cmd, "SERVER", 5)) {
		handle_server_ip(par, parnum);
	} else if (!strncmp((char*)cmd, "S-QUERY", 7)) {\
		if (parnum < 4) {
			return;
		}
		if (!strncmp((char*)par[1], "FB", 2)) {
			id = atoi((char*)par[2]);
			unlock_after_reboot(id);
		}
	} else if (!strncmp((char*)cmd, "OPEN", 6)) {
		//user_call_moto_start();
	} else if (!strncmp((char *)cmd, "RESET", 6)) {
		 //NVIC_SystemReset();	 
     //NVIC_SystemReset();	 
	}
}
		
static void parse_one_command(uint8_t *pstart, uint8_t *pend)
{
	uint8_t *imei;
	uint8_t *cmd;
	uint8_t *pcur;
	int32_t i;
	uint8_t *par[9];

	*pend = '\0';
	imei = (uint8_t *)strchr((char*)pstart, ',');
	if (imei == NULL) {
		return;
	}
	*imei = '\0';
	imei += 1;

	cmd = (uint8_t*) strchr((char*)imei, ',');
	if (cmd == NULL) {
		return;
	}
	*cmd='\0';
	cmd+=1;

	pcur = cmd;
	for (i = 0; i < 8; i++) {
		par[i] = (uint8_t*)strchr((char*)pcur, ',');
		if (par[i] == NULL) {
			break;
		}
		*par[i] = '\0';
		par[i]++;
		pcur = par[i];
	}

	for (i = 0; i < 8; i++) {
		if (par[i] != NULL) {
		} else {
			break;
		}
	}
	
	process_one_command(imei, cmd, par, i);
	return;
}


static void handle_TCP_protocal_receive(void)
{
	TCPFramePack *pFrame = NULL;
	uint8_t *pFramebuf = NULL;
	while (TCP_pack_length(&gTCPRecvPackFifo) > 0) {
		TCP_pack_get(&gTCPRecvPackFifo, &pFrame);
		if (pFrame) {
			pFramebuf = (uint8_t*)pFrame + 1;
			parse_one_command(pFramebuf, pFramebuf + pFrame->len);
			free((void*)pFrame);
			print_addr("free7", pFrame);
		}
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
				if (gTCPProtocalStatus == TCP_SEND_WAITACK) {	//timeout
					if (gSendRetryTimes++ < MAX_RETRY_SEND_TIME) {
						sendTCPPackFrame(gTCPCurSendFrame);
						etimer_set(&gTCPSendRetryEtimer, ((unsigned long)TCP_SEND_WAIT_ACK * CLOCK_SECOND));
					} else {
						if (gTCPCurSendFrame != NULL) {
							free((void*)gTCPCurSendFrame);
							print_addr("free1", gTCPCurSendFrame);
						}
						gTCPCurSendFrame = NULL;
						send_next_TCP_protocal_command();					
					}
				}
				break;
				
			case EV_TCP_SEND_OVER:
				if (gTCPCurSendFrame != NULL) {
					free((void*)gTCPCurSendFrame);
					print_addr("free2", gTCPCurSendFrame);
				}
				gTCPCurSendFrame = NULL;
				send_next_TCP_protocal_command();	
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


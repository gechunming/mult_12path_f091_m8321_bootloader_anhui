
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_it.h"
#include "tcp_htbt.h"
#include "gsm_test.h"

#define HTBT_WAIT_TIME   (15 * CLOCK_SECOND)
#define HTBT_INTERVAL_TIME  (300 * CLOCK_SECOND)
//#define HTBT_INTERVAL_TIME  (10 * CLOCK_SECOND)
#define HTBT_RETRY_INTERVAL_TIME  (30 * CLOCK_SECOND)
#define HTBT_TIMEOUT_MAX_TIMES  3

/*
*if detect 3 times disconnect but still no htbt back, then reset the device
*/
#define MAX_DISCONNECT_COUNT 3
static uint8_t mDisconnectCount = 0;

enum HTBTStatus {
	HTBT_STATUS_IDLE,
	HTBT_STATUS_WAIT_ACK
};

enum HTBTStatus mHtbtStatus = HTBT_STATUS_IDLE;
static uint8_t mHtbtTimeoutCount = 0;

PROCESS(process_tcp_htbt, "htbt process");
static struct etimer gTcpHtbtTimer;


void tcp_send_htbt(void);
extern struct process process_at_protocal;

static void handle_htbt_timeout(void)
{
	switch(mHtbtStatus) {
		case HTBT_STATUS_IDLE:
			tcp_send_htbt();
			mHtbtStatus = HTBT_STATUS_WAIT_ACK;
			etimer_set(&gTcpHtbtTimer, HTBT_WAIT_TIME);
			break;
		
		case HTBT_STATUS_WAIT_ACK:
			if (mHtbtTimeoutCount >= HTBT_TIMEOUT_MAX_TIMES) {
				process_post(&process_at_protocal, MODEM_EVENT_DISCONNECT, NULL);
				mHtbtTimeoutCount = 0x00;
				mHtbtStatus = HTBT_STATUS_IDLE;
				etimer_set(&gTcpHtbtTimer, HTBT_INTERVAL_TIME + HTBT_RETRY_INTERVAL_TIME * 2);
				
				mDisconnectCount++;
				if (mDisconnectCount > MAX_DISCONNECT_COUNT) {
					 NVIC_SystemReset();
					 NVIC_SystemReset();
					 NVIC_SystemReset();
				}
			} else {
				mHtbtTimeoutCount++;
				etimer_set(&gTcpHtbtTimer, HTBT_RETRY_INTERVAL_TIME);
				mHtbtStatus = HTBT_STATUS_IDLE;
			}
			break;
		
		default:
			break;
	}
}
PROCESS_THREAD(process_tcp_htbt, ev, data)
{

  PROCESS_BEGIN();

	etimer_set(&gTcpHtbtTimer, HTBT_INTERVAL_TIME);
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_htbt_timeout();
				break;
			
			case PROCESS_EVENT_TCP_HTBT_RECV:
				etimer_set(&gTcpHtbtTimer, HTBT_INTERVAL_TIME);
				mHtbtStatus = HTBT_STATUS_IDLE;
				mHtbtTimeoutCount = 0;
				mDisconnectCount = 0;
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


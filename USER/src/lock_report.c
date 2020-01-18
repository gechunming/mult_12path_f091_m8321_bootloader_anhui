#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "string.h"
#include "user_usart1.h"

#include "lock_report.h"

#define LOCK_REPORT_RETRY_INTERVAL_TIME  (1 * 60 * CLOCK_SECOND)
#define LOCK_REPORT_TIMEOUT_MAX_TIMES  3

/*
*if detect 3 times disconnect but still no LOCK_REPORT back, then reset the device
*/
#define MAX_DISCONNECT_COUNT 3

enum LOCK_REPORTStatus {
	LOCK_REPORT_STATUS_IDLE,
	LOCK_REPORT_STATUS_WAIT_ACK
};

static const char *mLockReasonInfo[] = {
	"FULL",
	"PWOFF",
	"CMD",
	"TEMP",
	"OVER",
	"LEAK",
	"OPEN"
};


static enum LOCK_REPORTStatus mLOCK_REPORTStatus = LOCK_REPORT_STATUS_IDLE;

PROCESS(process_tcp_LOCK_REPORT, "LOCK_REPORT process");
static struct etimer gTcpLOCK_REPORTTimer;

static enum LOCK_REASON mCurrentLockReason = LOCK_REASON_LOCKCMD;
static uint32_t mSubTotal = 0;
static uint32_t mTotal = 0;
static uint8_t mLOCK_REPORTTimeoutCount = 0;

int TCP_protocal_send(uint32_t len, uint8_t *param);
extern uint8_t IMEI[24];
void user_lock_report(enum LOCK_REASON reson, uint32_t subtotal, uint32_t total)
{
	uint8_t sbuf[64];
	
	if (mLOCK_REPORTStatus != LOCK_REPORT_STATUS_IDLE) {
		print_string("lock is sending busy");
		return;
	}
	
	mLOCK_REPORTStatus = LOCK_REPORT_STATUS_WAIT_ACK;
	mSubTotal = subtotal;
	mTotal = total;
	mLOCK_REPORTTimeoutCount = 1;
	mCurrentLockReason = reson;
	
	//send once
	snprintf((char*)sbuf, 63, (const char *)"*ZB,%s,LOCK,01,%s,%d,%d,112#", (char*)IMEI, mLockReasonInfo[reson], subtotal, total);
	TCP_protocal_send(strlen((const char*)sbuf), sbuf);
	
	PROCESS_CONTEXT_BEGIN(&process_tcp_LOCK_REPORT);
	etimer_set(&gTcpLOCK_REPORTTimer, LOCK_REPORT_RETRY_INTERVAL_TIME);
	PROCESS_CONTEXT_END(&process_tcp_LOCK_REPORT);
}
extern float mBatteryT;
void user_power_report(uint32_t subtotal, uint32_t power)
{
	uint8_t sbuf[64];
	//send once
	snprintf((char*)sbuf, 63, (const char *)"*ZB,%s,ELEC,01,FB,%d,%d,%d,0#", (char*)IMEI, subtotal, power,(int)mBatteryT);
	TCP_protocal_send(strlen((const char*)sbuf), sbuf);
}
static void user_lock_resend_report()
{
	uint8_t sbuf[64];
	//send once
	snprintf((char*)sbuf, 63, "*ZB,%s,LOCK,01,%s,%d,%d,112#", (char*)IMEI, mLockReasonInfo[mCurrentLockReason], mSubTotal, mTotal);
	TCP_protocal_send(strlen((char*)sbuf), sbuf);
}

void user_lock_charger_full_report(void)
{
	uint8_t sbuf[64];
	
	//send once
	snprintf((char*)sbuf, 63, (const char *)"*ZB,%s,CMPE,01,0#", (char*)IMEI);
	TCP_protocal_send(strlen((const char*)sbuf), sbuf);
}

static void handle_LOCK_REPORT_timeout(void)
{
	switch(mLOCK_REPORTStatus) {
		case LOCK_REPORT_STATUS_IDLE:
			break;
		
		case LOCK_REPORT_STATUS_WAIT_ACK:
			if (mLOCK_REPORTTimeoutCount > LOCK_REPORT_TIMEOUT_MAX_TIMES) {
				print_string("lock send failed");
				mLOCK_REPORTTimeoutCount = 0x00;
				mLOCK_REPORTStatus = LOCK_REPORT_STATUS_IDLE;
			} else {
				mLOCK_REPORTTimeoutCount++;
				user_lock_resend_report();
				etimer_set(&gTcpLOCK_REPORTTimer, LOCK_REPORT_RETRY_INTERVAL_TIME);
			}
			break;
		
		default:
			break;
	}
}
PROCESS_THREAD(process_tcp_LOCK_REPORT, ev, data)
{

  PROCESS_BEGIN();

//	etimer_set(&gTcpLOCK_REPORTTimer, LOCK_REPORT_RETRY_INTERVAL_TIME);
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_LOCK_REPORT_timeout();
				break;
			
			case PROCESS_EVENT_LOCK_REPORT_RECV:
				print_string("got lock report ack");
				mLOCK_REPORTStatus = LOCK_REPORT_STATUS_IDLE;
				mLOCK_REPORTTimeoutCount = 0;
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


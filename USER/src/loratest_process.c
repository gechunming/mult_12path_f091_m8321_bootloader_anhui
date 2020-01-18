#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "contiki.h"
#include "sx1276_spi.h"
#include "user_delay.h"

#include "contiki.h"


PROCESS(process_lora_test, "loratest process");
void user_call_send_lora_data( uint8_t cmd, uint8_t *data, uint8_t len);
static struct etimer mEtimer;

extern struct process process_sx1278;

void handle_lora_timeout(void)
{
	static uint8_t mLora = 0;
	uint8_t adata[1];
	
	if (mLora) {
			adata[0] = 1;
			mLora = 0;
		
	} else {
			adata[0] = 0;
			mLora = 1;
			//user_call_send_lora_data(0x01, adata, 1);
	}
	user_call_send_lora_data(0x01, adata, 1);
}

PROCESS_THREAD(process_lora_test, ev, data)
{

  PROCESS_BEGIN();
	etimer_set(&mEtimer, CLOCK_SECOND*3);
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_lora_timeout();
				etimer_set(&mEtimer, CLOCK_SECOND*1);
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}

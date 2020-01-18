#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "doorlock.h"

PROCESS(process_doordet, "lock det process");

static struct etimer mEtimer;

void DelayMs(uint32_t nTime);
void init_doorlock_gpio();

extern struct process process_doorlock;
static void handle_doordet_timeout(void)
{
	process_post(&process_doorlock, EVENT_DOOR_STATUS_CHANGED, NULL);
}

PROCESS_THREAD(process_doordet, ev, data)
{

  PROCESS_BEGIN();
	
  init_doorlock_gpio();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				etimer_set(&mEtimer, CLOCK_SECOND *20/1000);	//20ms
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_doordet_timeout();
				break;
		
			default:
				break;
		}
  }

  PROCESS_END();
}




void EXTI0_1_IRQHandler(void)
{
		process_poll(&process_doordet);
		EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1);
}

extern struct process process_moto_ctrl;
void EXTI2_3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
		process_poll(&process_moto_ctrl);
		EXTI_ClearITPendingBit(EXTI_Line2);
	} else {
		process_poll(&process_doordet);
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

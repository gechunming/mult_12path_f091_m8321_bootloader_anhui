#include "contiki.h"
#include "stdbool.h"
#include "stm32f0xx.h"
PROCESS(process1, "ETimer x Timer x STimer Process");

PROCESS_THREAD(process1, ev, data)
{
  static struct etimer timer_etimer;
	static bool g_pin = false;
  PROCESS_BEGIN();

  while(1) {
    etimer_set(&timer_etimer, CLOCK_SECOND*2);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		
		if (g_pin) {
			GPIO_SetBits(GPIOC, GPIO_Pin_9);
			g_pin = false;
		} else {
			g_pin = true;
			GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		}
  }

  PROCESS_END();
}

PROCESS(process2, "ETimer x Timer x STimer Process");

PROCESS_THREAD(process2, ev, data)
{
  static struct etimer timer_etimer;
	static bool g_pin = false;
  PROCESS_BEGIN();

  while(1) {
    etimer_set(&timer_etimer, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		
		if (g_pin) {
			GPIO_SetBits(GPIOC, GPIO_Pin_8);
			g_pin = false;
		} else {
			g_pin = true;
			GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
  }

  PROCESS_END();
}
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "contiki.h"
#include "stm32f0xx.h"
static void watchdog_init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_256);

  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
  IWDG_SetReload(0xfff);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}

PROCESS(process_wdt, "wdt");

PROCESS_THREAD(process_wdt, ev, data) 
{
	static struct etimer etimer;
	
	PROCESS_BEGIN();
	
	watchdog_init();
	
	while(1) {
		etimer_set(&etimer, CLOCK_SECOND*16);
		PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_TIMER:
			  IWDG_ReloadCounter();
				break;
				
			default:
				break;
		}
	}
	PROCESS_END();
}

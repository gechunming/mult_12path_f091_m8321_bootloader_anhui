#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "tcp_protocal.h"
#include <stdio.h>
#include "saferopemachine.h"

/*
vibrate detected use gpb2

*/
extern struct process process_safemachine;
PROCESS(process_vibrate, "vibrate");

static struct etimer gVibrateTimer;

enum ProcessVibrateEvent {
	ProcessVibrateEventOcurr,
};

static void init_vibrate(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	/* init every gpio */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  /* Connect EXTI Line to PXX pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	
	/* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0C;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static bool enablevibirq(bool en)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = en?ENABLE:DISABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void EXTI2_3_IRQHandler(void)
{
	//irqChannelEnable(EXTI2_3_IRQn, false);
	enablevibirq(false);
	process_post(&process_vibrate, ProcessVibrateEventOcurr, NULL);
	EXTI_ClearITPendingBit(EXTI_Line2 | EXTI_Line3);		//only clear it 
	
}

PROCESS_THREAD(process_vibrate, ev, data)
{

  PROCESS_BEGIN();
	
	init_vibrate();
	
  while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case ProcessVibrateEventOcurr: //key press detect
				print_string("vib\r\n");
				etimer_set(&gVibrateTimer, CLOCK_SECOND * 20);
				process_post(&process_safemachine, SAFE_MACHINE_EVENT_VIBRATE, NULL);
				break;
			
			case PROCESS_EVENT_TIMER:
				EXTI_ClearITPendingBit(EXTI_Line2 | EXTI_Line3);		//only clear it 
				//irqChannelEnable(EXTI2_3_IRQn, true);
			enablevibirq(true);
			print_string("vib en\r\n");
				break;

			default:
				break;
		}
  }

  PROCESS_END();
}


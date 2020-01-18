#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "tcp_protocal.h"
#include <stdio.h>
#include "bmp180.h"

#include "cablelock.h"
#include "saferopemachine.h"

extern struct process process_safemachine;
/*
cable lock init,
now use GPB1 for cable detect
*/
static CableLockMode  gCableLockMode = CABLE_LOCK_MODE_UNLOCKED;  

PROCESS(process_cable_lock, "gpio cable lock");
static struct etimer gCableRecheckTimer;


static void initCableLock(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	/* init every gpio */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  /* Connect EXTI Line to PXX pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);
	
	/* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	/*
	read gpio key value
	*/
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == Bit_SET) {
		gCableLockMode = CABLE_LOCK_MODE_LOCKED;
	} else {
		gCableLockMode = CABLE_LOCK_MODE_UNLOCKED;
	}
}

CableLockMode getCableLockModeAlive(void)
{
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == Bit_SET) {
		return CABLE_LOCK_MODE_LOCKED;
	} else {
		return CABLE_LOCK_MODE_UNLOCKED;
	}
}
void SleepAfterMs(uint32_t ms);
static void recheck_cablelock_mode(void)
{
//	uint8_t buf[10] = {0x00};
	CableLockMode curmode = CABLE_LOCK_MODE_UNLOCKED;
	
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == Bit_SET) {
		curmode = CABLE_LOCK_MODE_LOCKED;
	} else {
		curmode = CABLE_LOCK_MODE_UNLOCKED;
	}
	if (curmode == gCableLockMode) {
		return;  //reenter this mode
	}
	gCableLockMode = curmode;
	if (gCableLockMode == CABLE_LOCK_MODE_LOCKED) {
		//GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	//	snprintf((char*)buf, 10, "ockOpened");
		//TCP_protocal_send('l', 9, buf, false);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		process_post(&process_safemachine, SAFE_MACHINE_EVENT_LOCKED, NULL);
	} else {
	//	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		//snprintf((char*)buf, 10, "ockClosed");
		//TCP_protocal_send('l', 9, buf, false);
		//stop_measure_altitude();
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		process_post(&process_safemachine, SAFE_MACHINE_EVENT_UNLOCKED, NULL);
	}
	SleepAfterMs(10000);
}

void route_Lock_IRQHandler(void)
{
	PROCESS_CONTEXT_BEGIN(&process_cable_lock);
	etimer_set(&gCableRecheckTimer, CLOCK_SECOND * 1/1000);
	PROCESS_CONTEXT_END(&process_cable_lock);
	
	//EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1);		//only clear it 
}


PROCESS_THREAD(process_cable_lock, ev, data)
{

  PROCESS_BEGIN();
	
  initCableLock();

  while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				recheck_cablelock_mode();
				break;
	
			case PROCESS_CABLE_EVENT_UNLOCK:
				if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET) {
					GPIO_SetBits(GPIOB, GPIO_Pin_0);
					GPIO_SetBits(GPIOB, GPIO_Pin_1);
					GPIO_ResetBits(GPIOB, GPIO_Pin_10);
				}
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}

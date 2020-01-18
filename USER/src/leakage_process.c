#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "user_usart1.h"
PROCESS(process_leakage, "leakage process");

static struct etimer mEtimer;
static struct etimer mJitterTimer;
/*
PB4 for irq detect
PB3 for power enable
*/

enum LEAKAGE_STATUS {
	LEAKAGE_STATUS_POWEROFF,
	LEAKAGE_STATUS_POWERON,
	LEAKAGE_STATUS_DETING,
	LEAKAGE_STATUS_DETED,
};

#define NEXT_WAIT_TIME_WHEN_DETECT_LEAKAGE (CLOCK_SECOND * 60 * 10)

static enum LEAKAGE_STATUS mLeakageStatus = LEAKAGE_STATUS_POWEROFF;

bool isLeakageNow(void)
{
	if (mLeakageStatus == LEAKAGE_STATUS_DETED) {
		return true;
	} else {
		return false;
	}
}
static void init_gpio(void)
{
	int i;
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* Enable SYSCFG clock */

	/* init every gpio */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* Connect EXTI Line to PXX pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
	
	/* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void enable_leakage_power(bool enable)
{
	if (enable) {
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	} else {
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	}
}

static void enable_leakage_irq(bool enable)
{
	print_int((uint8_t*)"irq enable", enable);
	if (enable) {
		EXTI_Line_Enable(EXTI_Line4, ENABLE);
	} else {
		EXTI_Line_Enable(EXTI_Line4, DISABLE);
	}
} 

#include "user_usart1.h"
#include "lock_pwr.h"
extern struct process process_lock_pwr;

static void handle_detected_leakage(void) 
{
	switch(mLeakageStatus) {
		case LEAKAGE_STATUS_DETING:
			mLeakageStatus = LEAKAGE_STATUS_DETED;
			print_string("leakage detecte");
			process_post(&process_lock_pwr, LOCK_PROGESS_ACTION_LEAKAGE, NULL);
			etimer_set(&mEtimer, NEXT_WAIT_TIME_WHEN_DETECT_LEAKAGE);
			break;
		
		default:
			break;
	}
	
	return;
}
#include "led2.h"
static void handle_timerout(void)
{
	print_int("leakage time", mLeakageStatus);
	switch(mLeakageStatus) {
		case LEAKAGE_STATUS_POWEROFF:
			enable_leakage_power(true);
			mLeakageStatus = LEAKAGE_STATUS_POWERON;
			etimer_set(&mEtimer, CLOCK_SECOND/10); //10ms will enable irq deteced
			break;
		
		case LEAKAGE_STATUS_POWERON:
			enable_leakage_irq(true);
			mLeakageStatus = LEAKAGE_STATUS_DETING;
			refresh_led_status();	
			//etimer_set(&mEtimer, CLOCK_SECOND * 60 * 60);			//1 hour poll io
			etimer_set(&mEtimer, CLOCK_SECOND  * 20);	
			break;
		
		case LEAKAGE_STATUS_DETING:
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) != Bit_RESET) {
				print_string("timeout leakage");
				process_poll(&process_leakage);
			}
			//etimer_set(&mEtimer, CLOCK_SECOND *20);
			break;
		
		case LEAKAGE_STATUS_DETED:
			//will change to next schedule
			enable_leakage_irq(false);
			enable_leakage_power(false);
			mLeakageStatus = LEAKAGE_STATUS_POWEROFF;
			etimer_set(&mEtimer, CLOCK_SECOND  * 4);
			break;
		
		default:
			break;
	}
}

PROCESS_THREAD(process_leakage, ev, data)
{

  PROCESS_BEGIN();
	init_gpio();
	enable_leakage_irq(false);
	enable_leakage_power(false);
	etimer_set(&mEtimer, CLOCK_SECOND * 4);
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				etimer_set(&mJitterTimer, CLOCK_SECOND/10);
				break;
			
			case PROCESS_EVENT_TIMER:
				if (data == &mEtimer) {
					handle_timerout();
				} else if (data == &mJitterTimer) {
					if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) {		//no leakage
					} else {
						//leakage detected
						handle_detected_leakage();
					}	
				}
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}




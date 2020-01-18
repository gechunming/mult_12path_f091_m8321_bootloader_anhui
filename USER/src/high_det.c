#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "doorlock.h"

PROCESS(process_highdet, "high det process");

static struct etimer mEtimer;

#define HIGH_DET_PIN_GROUP  GPIOA
#define HIGH2_DET_PIN	GPIO_Pin_5
#define HIGH3_DET_PIN	GPIO_Pin_6
#define HIGH1_DET_PIN	GPIO_Pin_7

enum HighModeStatus {
	HighModeNoDetected,
	HighModeDeting,
	HighModeDetected,
};

static enum HighModeStatus mHighMode = HighModeNoDetected;

void DelayMs(uint32_t nTime);
void init_highdet_gpio()
{
	uint32_t lockstatus = 0;
	static bool lockIrqInit = false;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	EXTI_InitTypeDef   EXTI_InitStructure;

	if (lockIrqInit){
		return;
	}
	lockIrqInit = true;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	//GPIOB0 - 9
		GPIO_InitStructure.GPIO_Pin = HIGH1_DET_PIN | HIGH2_DET_PIN | HIGH3_DET_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line to PXX pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
	
		/* Configure EXTI2 line */
		EXTI_InitStructure.EXTI_Line =  EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
			
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
			
		DelayMs(10);
		
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH2_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH1_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH3_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
	
		if (lockstatus) {
			mHighMode = HighModeDetected;
		} else {
			mHighMode = HighModeNoDetected;
		}
}
   
void send_highdet_status(uint8_t status);
uint16_t user_get_high_det_status()
{
	return mHighMode;
}
static void handle_highdet_status_timeout(void)
{
		uint32_t lockstatus = 0;
	
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH2_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH1_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH3_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		
		if (lockstatus) {
			if (mHighMode != HighModeNoDetected) {
				etimer_set(&mEtimer, CLOCK_SECOND * 15);
				mHighMode = HighModeDetected;
				send_highdet_status(0x01);
			}
		} else {
			if (mHighMode != HighModeNoDetected) {
				mHighMode = HighModeNoDetected;
				send_highdet_status(0x00);
			}
		}	
}
static void handle_highdet_status_change(void)
{
		uint32_t lockstatus = 0;
	
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH2_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH1_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
		if (GPIO_ReadInputDataBit(HIGH_DET_PIN_GROUP, HIGH3_DET_PIN) == Bit_RESET) {
			lockstatus = 0x01;
		}
	
		if (lockstatus) {
			if (mHighMode == HighModeNoDetected) {
				etimer_set(&mEtimer, CLOCK_SECOND * 5);
				mHighMode = HighModeDeting;
			}
		} else {
			if (mHighMode != HighModeNoDetected) {
				if (mHighMode == HighModeDetected) {
					send_highdet_status(0x00);
				}
				mHighMode = HighModeNoDetected;
				etimer_stop(&mEtimer);
			}
		}	
}

PROCESS_THREAD(process_highdet, ev, data)
{

  PROCESS_BEGIN();
	
	init_highdet_gpio();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				//etimer_set(&mEtimer, CLOCK_SECOND *20/1000);	//20ms
				handle_highdet_status_change();
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_highdet_status_timeout();
				break;
		
			default:
				break;
		}
  }

  PROCESS_END();
}

extern struct process process_moto_ctrl;
extern struct process process_doordet;
void EXTI4_15_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		process_poll(&process_highdet);
		EXTI_ClearITPendingBit(EXTI_Line5);
	}  else if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		process_poll(&process_highdet);
		EXTI_ClearITPendingBit(EXTI_Line6);
	} else if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		process_poll(&process_highdet);
		EXTI_ClearITPendingBit(EXTI_Line7);
	} else if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
		process_poll(&process_doordet);
		EXTI_ClearITPendingBit(EXTI_Line15);
	} else if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		process_poll(&process_moto_ctrl);
		EXTI_ClearITPendingBit(EXTI_Line10);
	} else if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		process_poll(&process_moto_ctrl);
		EXTI_ClearITPendingBit(EXTI_Line11);
	} else {
		EXTI_ClearITPendingBit(EXTI_Line4  | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15);
	}
}

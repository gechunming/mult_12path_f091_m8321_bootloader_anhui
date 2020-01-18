#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "doorlock.h"

PROCESS(process_doorlock, "led process");

void user_pwm_timer_init(void);
void beep_enable(bool enable);

enum DoorStatus {
	DoorStatusClosed = 0x00,
	DoorStatusOpend, 
	DoorStatusUnkown,
};       
volatile enum DoorStatus mDoorStatus = DoorStatusUnkown;

static struct etimer mEtimer;
 
/*
GPIOA1   GPA0
GPIOF0   PC15
*/
#define LOCK1_CTRL_PIN_GROUP  GPIOA
#define LOCK2_CTRL_PIN_GROUP	GPIOF

#define LOCK1_CTRL_PIN  GPIO_Pin_1
#define LOCK2_CTRL_PIN	GPIO_Pin_0

#define LOCK1_DET_PIN_GROUP  GPIOA
#define LOCK2_DET_PIN_GROUP  GPIOC
#define LOCK1_DET_PIN	GPIO_Pin_0
#define LOCK2_DET_PIN	GPIO_Pin_15

void DelayMs(uint32_t nTime);
void init_doorlock_gpio()
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
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	//GPIOB0 - 9
		GPIO_InitStructure.GPIO_Pin = LOCK1_DET_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(LOCK1_DET_PIN_GROUP, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = LOCK2_DET_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(LOCK2_DET_PIN_GROUP, &GPIO_InitStructure);

	  /* Connect EXTI Line to PXX pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource15);
	
		/* Configure EXTI2 line */
		EXTI_InitStructure.EXTI_Line =  EXTI_Line0 | EXTI_Line15;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		DelayMs(10);
		
		if (GPIO_ReadInputDataBit(LOCK2_DET_PIN_GROUP, LOCK2_DET_PIN) == Bit_RESET) {
			//closed
			lockstatus = 0x01;
		} else {
			lockstatus = 0x00;
		}
		
		if (GPIO_ReadInputDataBit(LOCK1_DET_PIN_GROUP, LOCK1_DET_PIN) == Bit_RESET) {
			lockstatus |= 0x02;
		} else {
			lockstatus &= ~(0x02);
		}
		if (lockstatus == 0x03) {
			mDoorStatus = DoorStatusClosed;
		} else if (lockstatus == 0x00) {
			mDoorStatus = DoorStatusOpend;
		} else {
			mDoorStatus = DoorStatusUnkown;
		}
}
 
static void doorlock_init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	int i;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);	
	 /* Configure PC10 and PC11 in output pushpull mode */

	GPIO_ResetBits(LOCK2_CTRL_PIN_GROUP, LOCK2_CTRL_PIN);
	GPIO_ResetBits(LOCK1_CTRL_PIN_GROUP, LOCK1_CTRL_PIN);
	
	GPIO_InitStructure.GPIO_Pin = LOCK1_CTRL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LOCK1_CTRL_PIN_GROUP, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LOCK2_CTRL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LOCK2_CTRL_PIN_GROUP, &GPIO_InitStructure);
	
	GPIO_ResetBits(LOCK2_CTRL_PIN_GROUP, LOCK2_CTRL_PIN);
	GPIO_ResetBits(LOCK1_CTRL_PIN_GROUP, LOCK1_CTRL_PIN);
}

void door_open()
{
//	if (index >= MAX_DOOR_LOCK) return;

	GPIO_SetBits(LOCK2_CTRL_PIN_GROUP, LOCK2_CTRL_PIN);
	GPIO_SetBits(LOCK1_CTRL_PIN_GROUP, LOCK1_CTRL_PIN);
	
	PROCESS_CONTEXT_BEGIN(&process_doorlock);
	etimer_set(&mEtimer, CLOCK_SECOND/2);
	PROCESS_CONTEXT_END(&process_doorlock);
}

void send_doorlock_status(uint8_t status);
static void handle_doorlock_timeout(void)
{
	GPIO_ResetBits(LOCK2_CTRL_PIN_GROUP, LOCK2_CTRL_PIN);
	GPIO_ResetBits(LOCK1_CTRL_PIN_GROUP, LOCK1_CTRL_PIN);
	
	//if (mDoorStatus != DoorStatusOpend) {
	//	send_doorlock_status(DoorStatusUnkown);
	//} else {
		send_doorlock_status(mDoorStatus);
	//}
}



static void handle_door_status_changed(void)
{
	uint32_t lockstatus = 0;
	
	if (GPIO_ReadInputDataBit(LOCK2_DET_PIN_GROUP, LOCK2_DET_PIN) == Bit_RESET) {
		//closed
		lockstatus = 0x01;
	} else {
		GPIO_ResetBits(LOCK2_CTRL_PIN_GROUP, LOCK2_CTRL_PIN);
		lockstatus = 0x00;
	}
		
	if (GPIO_ReadInputDataBit(LOCK1_DET_PIN_GROUP, LOCK1_DET_PIN) == Bit_RESET) {
		lockstatus |= 0x02;
	} else {
		GPIO_ResetBits(LOCK1_CTRL_PIN_GROUP, LOCK1_CTRL_PIN);
		lockstatus &= ~(0x02);
	}
	if (lockstatus == 0x03) {
		if (mDoorStatus != DoorStatusClosed) {
			mDoorStatus = DoorStatusClosed;
			send_doorlock_status(mDoorStatus);
		}
	} else if (lockstatus == 0x00) {
		if (mDoorStatus != DoorStatusOpend) {
			mDoorStatus = DoorStatusOpend;
			send_doorlock_status(mDoorStatus);
		}
	} else {
		//send_doorlock_status(0x04);
		mDoorStatus = DoorStatusUnkown;
	}
}

uint16_t user_get_door_lock_status()
{
	handle_door_status_changed();
	return mDoorStatus;
}
void init_doorlock_gpio();
PROCESS_THREAD(process_doorlock, ev, data)
{

  PROCESS_BEGIN();
	
	doorlock_init();
	init_doorlock_gpio();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				//handle_door_status_changed();
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_doorlock_timeout();
				break;
			
			case EVENT_DOOR_STATUS_CHANGED:
				handle_door_status_changed();
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}

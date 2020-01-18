#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>

PROCESS(process_moto_ctrl, "moto ctrl process");

static struct etimer mEtimer;

#define MOTO_CTRL_OPEN_PIN		GPIO_Pin_11
#define MOTO_CTRL_CLOSE_PIN		GPIO_Pin_8

#define FAN_CTRL_PIN		GPIO_Pin_14
#define LIGHT_CTRL_PIN	GPIO_Pin_15

#define FINGER_PWR_PIN		GPIO_Pin_13
#define ICCARD_PWR_PIN		GPIO_Pin_14
#define FINGER_PWR_GROUP  GPIOC

#define MOTO_OPEN_DET_PIN			GPIO_Pin_10
#define MOTO_CLOSE_DET_PIN		GPIO_Pin_11

enum PROCESS_ACTION {
	PROCESS_ACTION_OPEN_SLIDE = 1,
	PROCESS_ACTION_CLOSE_SLIDE,
};
enum SLIDE_DOOR_STATUS {
	SLIDE_DOOR_STATUS_OPEN = 1,
	SLIDE_DOOR_STATUS_CLOSE,
	SLIDE_DOOR_STATUS_UNKOWN,
};

enum MOTO_DRIVER_MODE {
	MOTO_DRIVER_IDLE = 0,
	MOTO_DRIVER_OPEN,
	MOTO_DRIVER_CLOSE,
};


static enum SLIDE_DOOR_STATUS mSlideDoorStatus = SLIDE_DOOR_STATUS_UNKOWN;

static enum MOTO_DRIVER_MODE mMotoDriverMode = MOTO_DRIVER_IDLE;

static enum SLIDE_DOOR_STATUS get_current_slidedoor_status(void);
void DelayMs(uint32_t nTime);
void send_slidedoor_status(uint8_t status) ;
static void init_moto_gpio()
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	 /* Configure PC10 and PC11 in output pushpull mode */

	GPIO_InitStructure.GPIO_Pin = MOTO_CTRL_OPEN_PIN | MOTO_CTRL_CLOSE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);
	GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
	
	GPIO_InitStructure.GPIO_Pin = FAN_CTRL_PIN | LIGHT_CTRL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, FAN_CTRL_PIN);
	GPIO_SetBits(GPIOB, LIGHT_CTRL_PIN);
	
	GPIO_InitStructure.GPIO_Pin = FINGER_PWR_PIN | ICCARD_PWR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(FINGER_PWR_GROUP, &GPIO_InitStructure);
	GPIO_SetBits(FINGER_PWR_GROUP, FINGER_PWR_PIN);
	GPIO_SetBits(FINGER_PWR_GROUP, ICCARD_PWR_PIN);
	/*
	init irq
	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = MOTO_OPEN_DET_PIN | MOTO_CLOSE_DET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Connect EXTI Line to PXX pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);
	
		/* Configure EXTI2 line */
		EXTI_InitStructure.EXTI_Line =  EXTI_Line10 | EXTI_Line11;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		DelayMs(10);
		mSlideDoorStatus = get_current_slidedoor_status();
}

static enum SLIDE_DOOR_STATUS get_current_slidedoor_status(void)
{
	uint8_t gpiovalue = 0x00;
	enum SLIDE_DOOR_STATUS curstatus = SLIDE_DOOR_STATUS_UNKOWN;
	
	if (GPIO_ReadInputDataBit(GPIOB, MOTO_OPEN_DET_PIN) == Bit_SET) {
		gpiovalue = 0x01;
	} else {
		gpiovalue = 0x00;
	}
	if (GPIO_ReadInputDataBit(GPIOB, MOTO_CLOSE_DET_PIN) == Bit_SET) {
		gpiovalue |= 0x02;
	}
	
	switch(gpiovalue) {
		case 1:
			curstatus = SLIDE_DOOR_STATUS_CLOSE;
			break;
		
		case 2:
			curstatus = SLIDE_DOOR_STATUS_OPEN;
			break;
		
		default:
			curstatus = SLIDE_DOOR_STATUS_UNKOWN;
			break;
	}
	return curstatus;
}

uint16_t user_get_current_slide_door_status() {
	return get_current_slidedoor_status();
}

static void process_call_open_slide()
{
	mSlideDoorStatus = get_current_slidedoor_status();
	if (mSlideDoorStatus == SLIDE_DOOR_STATUS_OPEN) {
		send_slidedoor_status(mSlideDoorStatus);
		return;
	}
	if (mMotoDriverMode == MOTO_DRIVER_CLOSE) {
		GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);		//must stop first
		GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
		mMotoDriverMode = MOTO_DRIVER_IDLE;
		DelayMs(500);
	}
	GPIO_ResetBits(GPIOA, MOTO_CTRL_OPEN_PIN);			//now start open
	//GPIO_ResetBits(GPIOB, MOTO_CTRL_CLOSE_PIN);
	etimer_set(&mEtimer, CLOCK_SECOND * 25);				//max run time
	mMotoDriverMode = MOTO_DRIVER_OPEN;
}

static void process_call_close_slide()
{
	mSlideDoorStatus = get_current_slidedoor_status();
	if (mSlideDoorStatus == SLIDE_DOOR_STATUS_CLOSE) {
		send_slidedoor_status(mSlideDoorStatus);
		return;
	}
	if (mMotoDriverMode == MOTO_DRIVER_OPEN) {
		GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);		//must stop first
		GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
		mMotoDriverMode = MOTO_DRIVER_IDLE;
		DelayMs(500);
	}
	GPIO_ResetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);			//now start open
	//GPIO_ResetBits(GPIOB, MOTO_CTRL_CLOSE_PIN);
	etimer_set(&mEtimer, CLOCK_SECOND * 25);				//max run time
	mMotoDriverMode = MOTO_DRIVER_CLOSE;
}

static void handle_door_status_changed()
{
	mSlideDoorStatus = get_current_slidedoor_status();
	if ((mMotoDriverMode == MOTO_DRIVER_CLOSE)
				&&(mSlideDoorStatus == SLIDE_DOOR_STATUS_CLOSE)) {
					GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);		//must stop first
					GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
					mMotoDriverMode = MOTO_DRIVER_IDLE;
					send_slidedoor_status(mSlideDoorStatus);
					return;
	} 
	if ((mMotoDriverMode == MOTO_DRIVER_OPEN) 
			&& (mSlideDoorStatus == SLIDE_DOOR_STATUS_OPEN)) {
				GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);		//must stop first
				GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
				mMotoDriverMode = MOTO_DRIVER_IDLE;
				send_slidedoor_status(mSlideDoorStatus);
				return;
			}
			
	if ((mMotoDriverMode != MOTO_DRIVER_IDLE) &&
		(mSlideDoorStatus == SLIDE_DOOR_STATUS_UNKOWN)) {
			send_slidedoor_status(mSlideDoorStatus);
			return;
		}
	if ((mSlideDoorStatus != SLIDE_DOOR_STATUS_UNKOWN)) {
		send_slidedoor_status(mSlideDoorStatus);
	}
}

static void process_call_timeout() 
{
	mSlideDoorStatus = get_current_slidedoor_status();
	GPIO_SetBits(GPIOA, MOTO_CTRL_OPEN_PIN);		//must stop first
	GPIO_SetBits(GPIOA, MOTO_CTRL_CLOSE_PIN);
	mMotoDriverMode = MOTO_DRIVER_IDLE;
}  

void user_call_slide_action(bool open)
{
	if (open) {
		process_post(&process_moto_ctrl, PROCESS_ACTION_OPEN_SLIDE, NULL);
	} else {
		process_post(&process_moto_ctrl, PROCESS_ACTION_CLOSE_SLIDE, NULL);
	}
}

void user_call_fan_action(bool open)
{
	if (open) {
		GPIO_SetBits(GPIOB, FAN_CTRL_PIN);
	} else {
		GPIO_ResetBits(GPIOB, FAN_CTRL_PIN);
	}
}

void user_call_finger_power(bool open)
{
	if (open) {
		GPIO_SetBits(FINGER_PWR_GROUP, FINGER_PWR_PIN);
	} else {
		GPIO_ResetBits(FINGER_PWR_GROUP, FINGER_PWR_PIN);
	}
}

void user_call_light_action(bool open)
{
	if (!open) {
		GPIO_SetBits(GPIOB, LIGHT_CTRL_PIN);
	} else {
		GPIO_ResetBits(GPIOB, LIGHT_CTRL_PIN);
	}
}
uint16_t user_call_get_light_status(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB, LIGHT_CTRL_PIN) == Bit_RESET) {
		return 1;
	} else {
		return 0;
	}
}

PROCESS_THREAD(process_moto_ctrl, ev, data)
{

  PROCESS_BEGIN();
	
	init_moto_gpio();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				//handle_door_status_changed();
				handle_door_status_changed();
				break;
			
			case PROCESS_EVENT_TIMER:
				process_call_timeout();
				break;
			
			case PROCESS_ACTION_OPEN_SLIDE:
				process_call_open_slide();
				break;
			
			case PROCESS_ACTION_CLOSE_SLIDE:
				process_call_close_slide();
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}



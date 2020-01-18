#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "led2.h"
extern struct process process_safemachine;

PROCESS(process_gpio_key, "gpio key process");
static struct etimer key_poll_timer;

#define __DEBUG  1
void print_string(char *str);
enum KeyStatus {
	KEY_UP,
	KEY_JITTER,
	KEY_DOWN,
};
enum KeyEventVal {
	KEY_EV_UP,
	KEY_EV_DOWN,
	KEY_EV_REP
};
enum KeyCode {
	KeyCode_1 = 1,
	KeyCode_2,
	KeyCode_3,
	KeyCode_4,
	KeyCode_5,
	KeyCode_6,
	KeyCode_7,
	KeyCode_8
};

#define KEY_JITTER_TIME 120 //20ms jitter time
#define KEY_FIRST_DOWN_TIME 200
#define KEY_REPEATER_TIME  800 //1s for repeat key

typedef struct _GpioKeyPin {
	uint16_t pin;
	GPIO_TypeDef* gpiox;
	uint32_t oldEventTime;
	enum KeyStatus status;
	enum KeyCode code;
} GpioKeyPin;

/*
IN1  PB1
IN2  PB0
IN3  PB14
IN4  PB13
IN5  PB12
IN6  PA8
IN7  PB15
*/
volatile GpioKeyPin gKeyArray[] = {
	{
		GPIO_Pin_5,
		GPIOB,
		0,
		KEY_UP,
		KeyCode_1,
	}
};

uint32_t getCurTicks(void);
uint32_t getTickTimePassed(uint32_t oldtime);
void DelayMs(uint32_t nTime);
/*
for key init
*/
static void initGpioKey(void)
{
	int i;
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */

	/* init every gpio */
	for (i = 0; i < (sizeof(gKeyArray)/sizeof(GpioKeyPin)); i++) {
		GPIO_InitStructure.GPIO_Pin = gKeyArray[i].pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(gKeyArray[i].gpiox, &GPIO_InitStructure);
	}
	DelayMs(10);
	
  /* Connect EXTI Line to PXX pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);	
	
	/* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
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
	for (i = 0; i < (sizeof(gKeyArray)/sizeof(GpioKeyPin)); i++) {
		if (GPIO_ReadInputDataBit(gKeyArray[i].gpiox, gKeyArray[i].pin) == Bit_RESET) {
			gKeyArray[i].status = KEY_DOWN;
			gKeyArray[i].oldEventTime = getCurTicks();
		} else {
			gKeyArray[i].status = KEY_UP;
		}
	}
}
void user_call_calibrate_power(void);

enum CalKeyMode {
	CalKeyModeIdle,
	CalKeyModePressOne,
	CalKeyModePressTwo
};

#define FIRST_KEY_TIME_INTVAL  (5 * CLOCK_SECOND)

static enum CalKeyMode mCalKeyMode = CalKeyModeIdle;
static struct etimer mCalKeyEtimer;

extern void poweronoff_charger(bool enable);
extern volatile bool mInCalMode;
void report_key_value(uint8_t keycode, uint8_t val) 
{
	if (keycode == KeyCode_1) {
		if (KEY_EV_DOWN == val) {
			//user_call_calibrate_power();
			switch(mCalKeyMode) {
				case CalKeyModeIdle:
					mCalKeyMode = CalKeyModePressOne;
					etimer_set(&mCalKeyEtimer, FIRST_KEY_TIME_INTVAL);
					mInCalMode = true;
					poweronoff_charger(true);
					led_status_set(LED_STATUS_ALWAYS_OFF);
					break;
				case CalKeyModePressOne:
					mCalKeyMode = CalKeyModePressTwo;
					user_call_calibrate_power();
					etimer_set(&mCalKeyEtimer, CLOCK_SECOND * 25);
					break;
				
				default:
					break;
			}
		}
	}
}

static void process_key_event(void)
{
	int i = 0;
	bool keydown;
	uint32_t nextwaketime = 0xfffffff;
	
	for (i = 0; i < (sizeof(gKeyArray)/sizeof(GpioKeyPin)); i++) {
		if (GPIO_ReadInputDataBit(gKeyArray[i].gpiox, gKeyArray[i].pin) == Bit_RESET) {
			keydown = true;
		} else {
			keydown = false;
		}
		switch (gKeyArray[i].status) {
			case KEY_UP:
				if (keydown) {
					gKeyArray[i].oldEventTime = getCurTicks();
					gKeyArray[i].status = KEY_JITTER;
					if (nextwaketime > KEY_JITTER_TIME) {
						nextwaketime = KEY_JITTER_TIME;
					}
				}
				break;
			
			case KEY_DOWN:
				if (!keydown) {
					gKeyArray[i].status = KEY_UP;
					report_key_value(gKeyArray[i].code, KEY_EV_UP);
				}
				break;
			
			case KEY_JITTER:
				if (keydown) {
					if (getTickTimePassed(gKeyArray[i].oldEventTime) > KEY_JITTER_TIME) {
						gKeyArray[i].status = KEY_DOWN;
						//report key down
						report_key_value(gKeyArray[i].code, KEY_EV_DOWN);
					}
				} else {
					gKeyArray[i].status = KEY_UP;
				}
				break;
			}
	}
	etimer_set(&key_poll_timer, ((unsigned long)nextwaketime * CLOCK_SECOND)/ 1000);
}

PROCESS_THREAD(process_gpio_key, ev, data)
{

  PROCESS_BEGIN();
	
  initGpioKey();

  while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				//print_string("key event poll\r\n");
				process_key_event();
				break;
			
			case PROCESS_EVENT_TIMER:
				//print_char('t');
				if (&mCalKeyEtimer == data) {
					mCalKeyMode = CalKeyModeIdle;
					//led_status_set(LED_STATUS_BLINK_TWO);
				} else {
					process_key_event();
				}
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


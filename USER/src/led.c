#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>

PROCESS(process_led, "led process");

void user_pwm_timer_init(void);
void beep_enable(bool enable);

typedef struct _LedStruct {
	GPIO_TypeDef *gpiox;
	uint16_t pin;
	uint16_t ontime;
	uint16_t offtime;
	uint8_t blinktimes;
	bool curstatus;
	bool setvalue;
	struct etimer etimer;
} LedStruct;

static LedStruct gLedStruct[] = {
	{
		GPIOB,
		GPIO_Pin_8,
	},
	{
		GPIOB,
		GPIO_Pin_9,
	}
};
void led_set(uint8_t index, uint8_t val);

void led_init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	int i;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);	
	 /* Configure PC10 and PC11 in output pushpull mode */
	for (i = 0; i < (sizeof(gLedStruct)/sizeof(LedStruct)); i++) {
		if (gLedStruct[i].gpiox) {
			GPIO_InitStructure.GPIO_Pin = gLedStruct[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(gLedStruct[i].gpiox, &GPIO_InitStructure);
			GPIO_SetBits(gLedStruct[i].gpiox, gLedStruct[i].pin);
		} else {
			//user_pwm_timer_init();
		//	beep_enable(false);
		}
		
		gLedStruct[i].blinktimes = 0;
		gLedStruct[i].ontime = 0;
		gLedStruct[i].offtime = 0;
		gLedStruct[i].curstatus = false;
		gLedStruct[i].setvalue = false;
	}
}


void led_set(uint8_t index, uint8_t val)
{
	if (index >= (sizeof(gLedStruct)/sizeof(LedStruct))) {
		return;
	}
	if (gLedStruct[index].gpiox) {
		if (val) {
			GPIO_ResetBits(gLedStruct[index].gpiox, gLedStruct[index].pin);
		} else {
			GPIO_SetBits(gLedStruct[index].gpiox, gLedStruct[index].pin);
		}
	} else {
	}
	gLedStruct[index].setvalue = val?true:false;
}
void led_blink(uint8_t index, uint8_t blinktimes, uint16_t ontime, uint16_t offtime)
{
	if (index >= (sizeof(gLedStruct)/sizeof(LedStruct))) return;
	gLedStruct[index].blinktimes = blinktimes;
	gLedStruct[index].offtime = offtime;
	gLedStruct[index].ontime = ontime;
	gLedStruct[index].curstatus = (GPIO_ReadOutputDataBit(gLedStruct[index].gpiox, gLedStruct[index].pin) == Bit_SET)?true:false;
	
	if (gLedStruct[index].curstatus) {
		if (gLedStruct[index].gpiox) {
			GPIO_ResetBits(gLedStruct[index].gpiox, gLedStruct[index].pin);
		}
	} else {
		if (gLedStruct[index].gpiox) {
			GPIO_SetBits(gLedStruct[index].gpiox, gLedStruct[index].pin);
		}
	}
	PROCESS_CONTEXT_BEGIN(&process_led);
	etimer_set(&gLedStruct[index].etimer, CLOCK_SECOND*ontime/1000);
	PROCESS_CONTEXT_END(&process_led);
}

static void handle_led_timeout(void)
{
	int i;
	
	for (i = 0; i < (sizeof(gLedStruct)/sizeof(LedStruct)); i++) {
		if (gLedStruct[i].blinktimes > 0) {
			if (etimer_expired(&gLedStruct[i].etimer)) {
				if (gLedStruct[i].curstatus) {
					gLedStruct[i].curstatus = false;
					if (gLedStruct[i].gpiox) {
						GPIO_SetBits(gLedStruct[i].gpiox, gLedStruct[i].pin);
					}
					etimer_set(&gLedStruct[i].etimer, gLedStruct[i].offtime * CLOCK_SECOND/1000);
				} else {
					gLedStruct[i].curstatus = true;
					if (gLedStruct[i].gpiox) {
						GPIO_ResetBits(gLedStruct[i].gpiox, gLedStruct[i].pin);
					}
					etimer_set(&gLedStruct[i].etimer, gLedStruct[i].ontime * CLOCK_SECOND/1000);
				}
				if (gLedStruct[i].blinktimes != 0xff) {
					gLedStruct[i].blinktimes--;
				}
			}
		} else {
			if (gLedStruct[i].setvalue) {
				if (gLedStruct[i].gpiox) {
					GPIO_ResetBits(gLedStruct[i].gpiox, gLedStruct[i].pin);
				}
			} else {
				if (gLedStruct[i].gpiox) {
					GPIO_SetBits(gLedStruct[i].gpiox, gLedStruct[i].pin);
				}
			}
		}
	}
}
PROCESS_THREAD(process_led, ev, data)
{

  PROCESS_BEGIN();
	
	led_init();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_led_timeout();
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


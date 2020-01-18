#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "led2.h"

PROCESS(process_led, "led process");

#define LED_BLUE_PIN  GPIO_Pin_9
#define LED_RED_PIN	GPIO_Pin_8



static enum LED_STATUS mLedStatus = LED_STATUS_ALWAYS_ON_ALL;
static struct etimer mBlinkTimer;
static bool mLightOn = false;

void led_init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	

	GPIO_InitStructure.GPIO_Pin = LED_RED_PIN | LED_BLUE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, LED_BLUE_PIN);
	GPIO_SetBits(GPIOB, LED_RED_PIN);
}

void led_status_set(enum LED_STATUS ledstatus)
{
	PROCESS_CONTEXT_BEGIN(&process_led);
	mLedStatus = ledstatus;
	switch(ledstatus) {
		case LED_STATUS_BLINK_TWO:
			GPIO_ResetBits(GPIOB, LED_RED_PIN);
			GPIO_SetBits(GPIOB, LED_BLUE_PIN);
			mLightOn = false;
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		case LED_STATUS_BLINK_RED:
			GPIO_ResetBits(GPIOB, LED_RED_PIN);
			GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			mLightOn = false;
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		case LED_STATUS_BLINK_BLUE:
			GPIO_ResetBits(GPIOB, LED_RED_PIN);
			GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			mLightOn = false;
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		case LED_STATUS_ALWAYS_ON_RED:
			GPIO_SetBits(GPIOB, LED_RED_PIN);
			GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			break;
		
		case LED_STATUS_ALWAYS_ON_BLUE:
			GPIO_ResetBits(GPIOB, LED_RED_PIN);
			GPIO_SetBits(GPIOB, LED_BLUE_PIN);
			break;
		
		case LED_STATUS_ALWAYS_ON_ALL:
			GPIO_SetBits(GPIOB, LED_RED_PIN);
			GPIO_SetBits(GPIOB, LED_BLUE_PIN);
			break;
		
		case LED_STATUS_ALWAYS_OFF:
		default:
			GPIO_ResetBits(GPIOB, LED_RED_PIN);
			GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			break;
		
	}
	PROCESS_CONTEXT_END(&process_led);
}
bool IsTcpConnected(void);
bool isLeakageNow(void);
bool isTemperatureHighNow(void);
bool isLockPowerFull(void);
bool isLockCharging(void);
extern volatile bool mInCalMode;
void refresh_led_status(void)
{
	if (mInCalMode) {
		return;
	}		
	if (isLeakageNow() || isTemperatureHighNow()){
		led_status_set(LED_STATUS_ALWAYS_ON_RED);
	} else if (IsTcpConnected()) {
		if (isLockPowerFull()) {
			led_status_set(LED_STATUS_BLINK_RED);
		} else if (isLockCharging()) {
			led_status_set(LED_STATUS_BLINK_BLUE);
		} else {
			led_status_set(LED_STATUS_BLINK_TWO);
		}
	} else {
		if (isLockPowerFull()) {
			led_status_set(LED_STATUS_BLINK_RED);
		} else if (isLockCharging()) {
			led_status_set(LED_STATUS_BLINK_BLUE);
		} else {
			led_status_set(LED_STATUS_ALWAYS_OFF);
		}
	}
}
static void handle_led_timeout(void)
{
	switch(mLedStatus) {
		case LED_STATUS_BLINK_TWO:
			mLightOn = !mLightOn;
			if (mLightOn) {
				GPIO_SetBits(GPIOB, LED_RED_PIN);
				GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			} else {
				GPIO_ResetBits(GPIOB, LED_RED_PIN);
				GPIO_SetBits(GPIOB, LED_BLUE_PIN);
			}
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		case LED_STATUS_BLINK_BLUE:
			mLightOn = !mLightOn;
			if (mLightOn) {
				GPIO_SetBits(GPIOB, LED_BLUE_PIN);
			} else {
				GPIO_ResetBits(GPIOB, LED_BLUE_PIN);
			}
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		case LED_STATUS_BLINK_RED:
			mLightOn = !mLightOn;
			if (mLightOn) {
				GPIO_SetBits(GPIOB, LED_RED_PIN);
			} else {
				GPIO_ResetBits(GPIOB, LED_RED_PIN);
			}
			etimer_set(&mBlinkTimer, BLINK_TIME);
			break;
		
		default:
			break;
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


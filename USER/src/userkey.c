#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>

PROCESS(process_user_key, "user key process");
static struct etimer key_poll_timer;

#define KEY_DEBOUNCE_TIME (CLOCK_SECOND/20)  //200ms
#define KEY_LONG_PRESS_TIME (CLOCK_SECOND * 1) //4 seconds for long press
#define KEY_LONG_PRESS_DEBOUND_TIME (3)  //5 seconds

static uint8_t g_key_press_number = 0x00;
enum {
	KEY_PRESSE_OCUR,			/* Detect key press, then use timer for check */
};


/*
GPIOB 2 for user key
*/
static void user_key_init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	
  /* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//void handle_lora_tx_status(bool sucess);
void tipple_led()
{
	static bool g_pin = false;
			if (g_pin) {
			GPIO_SetBits(GPIOC, GPIO_Pin_9);
			g_pin = false;
		} else {
			g_pin = true;
			GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		}
		//PrepareTxFrame();
		//handle_lora_tx_status(true);
}

void env_scan_addr_set_start();
void tipple_led2()
{
	static bool g_pin = false;
			if (g_pin) {
			GPIO_SetBits(GPIOC, GPIO_Pin_8);
			g_pin = false;
		} else {
			g_pin = true;
			GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
		//PrepareTxFrame();
		env_scan_addr_set_start();
}


PROCESS_THREAD(process_user_key, ev, data)
{

  PROCESS_BEGIN();
	
  user_key_init();

  while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == Bit_SET)  {
					etimer_set(&key_poll_timer, KEY_DEBOUNCE_TIME);
					g_key_press_number = 0x00;
				} else {
					etimer_stop(&key_poll_timer);
				}
				break;
			
			case PROCESS_EVENT_TIMER:
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == Bit_SET)  {
					g_key_press_number++;
					if (g_key_press_number == 1) {
						//key pressed
						tipple_led();
					} else if (g_key_press_number == KEY_LONG_PRESS_DEBOUND_TIME) {
						//key long press
						tipple_led2();
					}
					etimer_set(&key_poll_timer, KEY_LONG_PRESS_TIME);
				} else {
					etimer_stop(&key_poll_timer);
					
				}
			
		}
  }

  PROCESS_END();
}


void EXTI2_3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)) {
			//key press got
			process_poll(&process_user_key);
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
    /* Clear the EXTI line 0 pending bit */
    //
  } else {
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

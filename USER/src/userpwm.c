#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
 
/*
use gpio8 gpio9 for pwm output
*/
#define USER_PWM1_PIN  GPIO_Pin_8
#define USER_PWN2_PIN	 GPIO_Pin_11

#define PWM_TIME_CLOCK_FREQ   47570  //17.57KHz

static uint16_t gtimeperioid = 0x00;

void user_pwm_timer_init(void)
{
	TIM_TimeBaseInitTypeDef TimTimeBaeStructure;
	TIM_OCInitTypeDef TimOcInitStructure;
	GPIO_InitTypeDef GpioInitStructure;
	gtimeperioid = (SystemCoreClock/PWM_TIME_CLOCK_FREQ) - 1;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	//gpio8 gpio9 config
	GpioInitStructure.GPIO_Pin = GPIO_Pin_8;
	GpioInitStructure.GPIO_Mode = GPIO_Mode_AF;
	GpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GpioInitStructure.GPIO_OType = GPIO_OType_PP;
	GpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GpioInitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TimTimeBaeStructure.TIM_Prescaler = 9;
	TimTimeBaeStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimTimeBaeStructure.TIM_Period = gtimeperioid;
	TimTimeBaeStructure.TIM_ClockDivision = 0;
	TimTimeBaeStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TimTimeBaeStructure);
	
	TimOcInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TimOcInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TimOcInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TimOcInitStructure.TIM_Pulse = (uint16_t) ((((uint32_t)gtimeperioid-1) * 500)/1000);
	TimOcInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TimOcInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TimOcInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TimOcInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM1, &TimOcInitStructure);
	
	//TimOcInitStructure.TIM_Pulse = (uint16_t) ((((uint32_t)gtimeperioid-1) * 125)/1000);
//	TIM_OC4Init(TIM1, &TimOcInitStructure);
	
	TIM_Cmd(TIM1, DISABLE);
	
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

void beep_enable(bool enable)
{
	if (enable) {
		TIM_Cmd(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
	} else {
		TIM_Cmd(TIM1, DISABLE);
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
	}
}
/*
void change_pwm_ouput_percent(uint8_t ch, uint8_t percent)
{
	uint16_t tick = (uint16_t)((((uint32_t)gtimeperioid-1) * percent)/100);
	switch(ch) {
		case 1:
			TIM_SetCompare1(TIM1, tick);
			break;
		case 2:
			TIM_SetCompare4(TIM1, tick);
			break;
		default:
			break;
	}
	return;
}
*/

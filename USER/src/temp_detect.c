/**
  ******************************************************************************
  * @file    useradc.c 
  * @author  karl(18653528604/gcmywode@126.com)
  * @version V1.0.9
  * @date    2018-08-08
  * @brief   Main program body
  ******************************************************************************
	* History:
	*
  *
  ******************************************************************************
  */
#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
#include "math.h"
/*
pa0 --> adc channel 0
pb0 ---> adc channel 8

*/
#define USER1_ADC_PIN 	GPIO_Pin_0

PROCESS(process_adc, "adc process");

static struct etimer gAdcScheduleTimer;		//will 20 seconds run once
static struct etimer gAdcSampleTimer;

static void adc_init(void)
{
	ADC_InitTypeDef AdcInitStructure;
	GPIO_InitTypeDef GpioInitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
	
	ADC_DeInit(ADC1);
	
	//gpioc clock enable
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//set GPC1 as analog input
	GpioInitStructure.GPIO_Pin = GPIO_Pin_0;
	GpioInitStructure.GPIO_Mode = GPIO_Mode_AN;
	GpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GpioInitStructure);
	
	ADC_StructInit(&AdcInitStructure);
	AdcInitStructure.ADC_Resolution = ADC_Resolution_12b;
	AdcInitStructure.ADC_ContinuousConvMode = DISABLE;
	AdcInitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	AdcInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	AdcInitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &AdcInitStructure);
	
	ADC_ChannelConfig(ADC1, ADC_Channel_8, ADC_SampleTime_239_5Cycles);
	
	ADC_GetCalibrationFactor(ADC1);
	
//	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
//	ADC_DMACmd(ADC1, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
		
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0xf;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
	
//	ADC_StartOfConversion(ADC1);
}


void user_adc_init(void)
{
	adc_init();
	
	//while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET);		//wait for dma transfer complete first time
}

#define BATTERY_SAMPLING_TIMES 4
#define BATTERY_LOW_VOLTAGE 0xDB0

volatile uint16_t adcValue[BATTERY_SAMPLING_TIMES] = {0x00};
static uint8_t adcCovTimes = 0x00;


void print_battary(uint16_t data);
#include "user_usart1.h"
#include "lock_pwr.h"
extern struct process process_lock_pwr;
float mBatteryT = 25.0;
bool isTemperatureHighNow(void)
{
	if (mBatteryT > 80.0) {
		//return true;
		//karl
		return false;
	} else {
		return false;
	}
}


static void report_battery_mode(void)
{
	uint32_t adcS = 0x00;
	uint32_t res = 0;
	uint8_t i;
	float Rt=0;
  float Rp=100000;
  float T2=273.15+25;
  float Bx=4311;
  float Ka=273.15;
  float vol=0;	
	
	for (i = 0; i < BATTERY_SAMPLING_TIMES; i++) {
		adcS += adcValue[i];
	}
	adcS = adcS / BATTERY_SAMPLING_TIMES;
	
	/*
	res = 100 *1000 * adcS / (4096 - adcS);
	
	t1 = (double)(log((double)res)/log((double)(100 * 1000)));
	t1 = t1/4311;
	t1 = t1 + 1/(273.15+25);
	t1 = 1/t1;
	t1 = t1 - 273.15;
	*/
  vol=(float)((adcS)*(2.8/4096));
  Rt=(2.8-vol)*100000/vol;
  mBatteryT=1/(1/T2+log(Rt/Rp)/Bx)-Ka+0.5;
	
	if (mBatteryT >80.0) {
		process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_TEMPERATURE_WARNING, NULL);
	}
}
void ADCx_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET) {
		adcValue[adcCovTimes] = ADC_GetConversionValue(ADC1);
	}
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC | ADC_IT_OVR);
	if (adcCovTimes < (BATTERY_SAMPLING_TIMES-1)) {
		adcCovTimes++;
		ADC_StartOfConversion(ADC1);
	} else {
		report_battery_mode();
	}
}
void SleepAfterMs(uint32_t ms);
PROCESS_THREAD(process_adc, ev, data)
{

  PROCESS_BEGIN();

	user_adc_init();
	adcCovTimes = 0x00;
	//ADC_StartOfConversion(ADC1);
	
	etimer_set(&gAdcScheduleTimer, CLOCK_SECOND * 5);
		
  while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_TIMER: //key press detect
				if (data == &gAdcScheduleTimer) {
					etimer_set(&gAdcScheduleTimer, CLOCK_SECOND * 10);
					
					etimer_set(&gAdcSampleTimer, CLOCK_SECOND * 1);			//1s后启动一次采集
				} else if (data == &gAdcSampleTimer) {
					adcCovTimes = 0x00;
					ADC_StartOfConversion(ADC1);
				}
				break;
		
			default:
				break;
		}
  }

  PROCESS_END();
}




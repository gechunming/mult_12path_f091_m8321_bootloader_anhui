#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <stdint.h>
 
/*
pa0 --> adc channel 0

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
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//set GPC1 as analog input
	GpioInitStructure.GPIO_Pin = GPIO_Pin_0;
	GpioInitStructure.GPIO_Mode = GPIO_Mode_AN;
	GpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GpioInitStructure);
	
	ADC_StructInit(&AdcInitStructure);
	AdcInitStructure.ADC_Resolution = ADC_Resolution_12b;
	AdcInitStructure.ADC_ContinuousConvMode = DISABLE;
	AdcInitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	AdcInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	AdcInitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &AdcInitStructure);
	
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
	
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

static bool gBatteryLowMode = false;
void print_battary(uint16_t data);
static void report_battery_mode(void)
{
	uint16_t voltage = 0x00;
	uint8_t i;
	
	for (i = 0; i < BATTERY_SAMPLING_TIMES; i++) {
		voltage += adcValue[i];
	}
	voltage = voltage / BATTERY_SAMPLING_TIMES;
	
	print_battary(voltage);
	
	if (gBatteryLowMode) {//in low battery mode
		if (voltage > (BATTERY_LOW_VOLTAGE + 100)) {
			gBatteryLowMode = false;
			led_set(0, 0);
		} 
	} else {
		if (voltage < BATTERY_LOW_VOLTAGE) {
			gBatteryLowMode = true;
			led_set(0, 1);
		}
		if (voltage < 0xccc) {
				print_string("power off\r\n");
			led_set(3, 0); //power off
		}
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
				if (etimer_expired(&gAdcScheduleTimer)) {
					etimer_set(&gAdcScheduleTimer, CLOCK_SECOND * 5*60);
					
					etimer_set(&gAdcSampleTimer, CLOCK_SECOND * 1);
					SleepAfterMs(2000);
					
				} else if (etimer_expired(&gAdcSampleTimer)) {
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



#include "stm32f0xx.h"
#include "stm32f0xx_spi.h"
#include "sx1276_spi.h"

#define TIME 1

void dellayus(unsigned int i)
{
	unsigned int j,k; 
	for (j = 0; j <10; j++){
		for (k = 0; k < i; k++);
	}
}

void SPI_Init1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	
	/* Enable the SPI periph */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SET_H();
}

void SpiInOut(unsigned char senddata)
{
	unsigned char i;
	
	for(i=0;i<8;i++)
	{
		dellayus(TIME);
		CLK_L();                                                        /* clk=0;                       */
		if((senddata<<i)&0x80){
			dellayus(TIME);
			SDI_H();                                                    /* set 1;                       */
		} else {
			dellayus(TIME);	
			SDI_L();                                                    /* set 0;                       */
		} 
		dellayus(TIME);	
		CLK_H();                                                        /* clk=1;                       */
	}
	dellayus(TIME);  
	CLK_L();
	SDI_L();                                                            /* set 0;                       */ 
}

unsigned char SpiRCVaByte(void)
{
	unsigned char i,temp;
	temp = 0;
	for(i=0;i<8;i++)
	{

		CLK_L();                                                        /* clk=0;                       */
		dellayus(TIME);   
		temp=(temp<<1);
		CLK_H();                                                         /* clk=1;                       */
		dellayus(TIME); 
		if(SDO_READ())
				temp++;                                                     /* set 1;                       */
			else
				temp = temp+0;
		dellayus(TIME); 
	}
	CLK_L();
	dellayus(TIME);
	return temp;
}



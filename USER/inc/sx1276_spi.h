#ifndef __SX1276_SPI_H__
#define __SX1276_SPI_H__


#include "stm32f0xx.h"
//cs
#define SET_L()  GPIOA->BRR = GPIO_Pin_12
#define SET_H()   GPIOA->BSRR = GPIO_Pin_12

//mosi
#define SDI_L()        GPIOA->BRR = GPIO_Pin_2
#define SDI_H()        GPIOA->BSRR = GPIO_Pin_2  

//clk
#define CLK_L()       GPIOA->BRR = GPIO_Pin_3
#define CLK_H()       GPIOA->BSRR = GPIO_Pin_3

//miso
#define SDO_READ()    ((((uint16_t)GPIOA->IDR)  & GPIO_Pin_1) != 0)


#endif

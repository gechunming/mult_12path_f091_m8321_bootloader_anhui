#include <stdio.h>
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_fifo.h"
#include "string.h"
#include "gsm_test.h"
#include "userflash.h"
#include "led2.h"

#define LED_BLUE_PIN_GROUP GPIOB
#define LED_BLUE_PIN  GPIO_Pin_2
#define LED_RED_PIN_GROUP  GPIOB
#define LED_RED_PIN	GPIO_Pin_10

//usart1 for rs485 used for upgrade
#define UART1_DMA_BUF 1024
uint8_t g_uart1_dmabuf[UART1_DMA_BUF];

#define UART1_USER_FIFO_BUF_SIZE 1024
uint8_t g_uart2_user_buf[UART1_USER_FIFO_BUF_SIZE];

static app_fifo_t                  m_rx_fifo;

volatile int mFirmwareSize = 0;


/*!< USART CR1 register clear Mask ((~(uint32_t)0xFFFFE6F3)) */
#define CR1_CLEAR_MASK            ((uint32_t)(USART_CR1_M | USART_CR1_PCE | \
                                              USART_CR1_PS | USART_CR1_TE | \
                                              USART_CR1_RE))

/*!< USART CR2 register clock bits clear Mask ((~(uint32_t)0xFFFFF0FF)) */
#define CR2_CLOCK_CLEAR_MASK      ((uint32_t)(USART_CR2_CLKEN | USART_CR2_CPOL | \
                                              USART_CR2_CPHA | USART_CR2_LBCL))

/*!< USART CR3 register clear Mask ((~(uint32_t)0xFFFFFCFF)) */
#define CR3_CLEAR_MASK            ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))

/*!< USART Interrupts mask */
#define IT_MASK                   ((uint32_t)0x000000FF)

void my_memcpy(uint8_t *dst, uint8_t *src, int len)
{
	int i = 0;
	for (i = 0 ;i < len; i++) {
		dst[i] = src[i];
	}
}

void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_AHB_PERIPH(RCC_AHBPeriph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    RCC->AHBENR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBENR &= ~RCC_AHBPeriph;
  }
}


void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2ENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1ENR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1ENR &= ~RCC_APB1Periph;
  }
}

/**
  * @brief  Initializes the GPIOx peripheral according to the specified 
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @note   GPIOE is available only for STM32F072.
  * @note   GPIOD is not available for STM32F031.   
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;

  /*-------------------------- Configure the port pins -----------------------*/
  /*-- GPIO Mode Configuration --*/
  for (pinpos = 0x00; pinpos < 0x10; pinpos++)
  {
    pos = ((uint32_t)0x01) << pinpos;

    /* Get the port pins position */
    currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

    if (currentpin == pos)
    {
      if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
      {
        /* Check Speed mode parameters */
        assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
        GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));

        /* Check Output mode parameters */
        assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

        /* Output mode configuration */
        GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));
      }

      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));

      GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));

      /* Pull-up Pull down resistor configuration */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
      GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));
    }
  }
}

void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;

  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}

void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the selected USART by setting the UE bit in the CR1 register */
    USARTx->CR1 |= USART_CR1_UE;
  }
  else
  {
    /* Disable the selected USART by clearing the UE bit in the CR1 register */
    USARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
  }
}

void USART_DMACmd(USART_TypeDef* USARTx, uint32_t USART_DMAReq, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 |= USART_DMAReq;
  }
  else
  {
    /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 &= (uint32_t)~USART_DMAReq;
  }
}
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0, presc = 0, pllclk = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        pllclk = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        pllclk = (HSE_VALUE / prediv1factor) * pllmull; 
      }
      RCC_Clocks->SYSCLK_Frequency = pllclk;      
      break;
    case 0x0C:  /* HSI48 used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI48_VALUE;
      break;
    default: /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
  }
  /* Compute HCLK, PCLK clocks frequencies -----------------------------------*/
  /* Get HCLK prescaler */
  tmp = RCC->CFGR & RCC_CFGR_HPRE;
  tmp = tmp >> 4;
  presc = APBAHBPrescTable[tmp]; 
  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

  /* Get PCLK prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE;
  tmp = tmp >> 8;
  presc = APBAHBPrescTable[tmp];
  /* PCLK clock frequency */
  RCC_Clocks->PCLK_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* ADCCLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_ADCSW) != RCC_CFGR3_ADCSW)
  {
    /* ADC Clock is HSI14 Osc. */
    RCC_Clocks->ADCCLK_Frequency = HSI14_VALUE;
  }
  else
  {
    if((RCC->CFGR & RCC_CFGR_ADCPRE) != RCC_CFGR_ADCPRE)
    {
      /* ADC Clock is derived from PCLK/2 */
      RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK_Frequency >> 1;
    }
    else
    {
      /* ADC Clock is derived from PCLK/4 */
      RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK_Frequency >> 2;
    }
    
  }

  /* CECCLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_CECSW) != RCC_CFGR3_CECSW)
  {
    /* CEC Clock is HSI/244 */
    RCC_Clocks->CECCLK_Frequency = HSI_VALUE / 244;
  }
  else
  {
    /* CECC Clock is LSE Osc. */
    RCC_Clocks->CECCLK_Frequency = LSE_VALUE;
  }

  /* I2C1CLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_I2C1SW) != RCC_CFGR3_I2C1SW)
  {
    /* I2C1 Clock is HSI Osc. */
    RCC_Clocks->I2C1CLK_Frequency = HSI_VALUE;
  }
  else
  {
    /* I2C1 Clock is System Clock */
    RCC_Clocks->I2C1CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
  }

  /* USART1CLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == 0x0)
  {
    /* USART1 Clock is PCLK */
    RCC_Clocks->USART1CLK_Frequency = RCC_Clocks->PCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW_0)
  {
    /* USART1 Clock is System Clock */
    RCC_Clocks->USART1CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW_1)
  {
    /* USART1 Clock is LSE Osc. */
    RCC_Clocks->USART1CLK_Frequency = LSE_VALUE;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW)
  {
    /* USART1 Clock is HSI Osc. */
    RCC_Clocks->USART1CLK_Frequency = HSI_VALUE;
  }
  
  /* USART2CLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == 0x0)
  {
    /* USART Clock is PCLK */
    RCC_Clocks->USART2CLK_Frequency = RCC_Clocks->PCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW_0)
  {
    /* USART Clock is System Clock */
    RCC_Clocks->USART2CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW_1)
  {
    /* USART Clock is LSE Osc. */
    RCC_Clocks->USART2CLK_Frequency = LSE_VALUE;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW)
  {
    /* USART Clock is HSI Osc. */
    RCC_Clocks->USART2CLK_Frequency = HSI_VALUE;
  }
  
  /* USART3CLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == 0x0)
  {
    /* USART Clock is PCLK */
    RCC_Clocks->USART3CLK_Frequency = RCC_Clocks->PCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW_0)
  {
    /* USART Clock is System Clock */
    RCC_Clocks->USART3CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW_1)
  {
    /* USART Clock is LSE Osc. */
    RCC_Clocks->USART3CLK_Frequency = LSE_VALUE;
  }
  else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW)
  {
    /* USART Clock is HSI Osc. */
    RCC_Clocks->USART3CLK_Frequency = HSI_VALUE;
  }
  
  /* USBCLK clock frequency */
  if((RCC->CFGR3 & RCC_CFGR3_USBSW) != RCC_CFGR3_USBSW)
  {
    /* USB Clock is HSI48 */
    RCC_Clocks->USBCLK_Frequency = HSI48_VALUE;
  }
  else
  {
    /* USB Clock is PLL clock */
    RCC_Clocks->USBCLK_Frequency = pllclk;
  }   
}

void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  uint32_t divider = 0, apbclock = 0, tmpreg = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;
  
  /* Disable USART */
  USARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
  
  /*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);
  
  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
  
  /* Write to USART CR2 */
  USARTx->CR2 = tmpreg;
  
  /*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);
  
  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
    USART_InitStruct->USART_Mode;
  
  /* Write to USART CR1 */
  USARTx->CR1 = tmpreg;
  
  /*---------------------------- USART CR3 Configuration -----------------------*/  
  tmpreg = USARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR3_CLEAR_MASK);
  
  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
  
  /* Write to USART CR3 */
  USARTx->CR3 = tmpreg;
  
  /*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  
  if (USARTx == USART1)
  {
    apbclock = RCC_ClocksStatus.USART1CLK_Frequency;
  }
  
  /* Determine the integer part */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
  {
    /* (divider * 10) computing in case Oversampling mode is 8 Samples */
    divider = (uint32_t)((2 * apbclock) / (USART_InitStruct->USART_BaudRate));
    tmpreg  = (uint32_t)((2 * apbclock) % (USART_InitStruct->USART_BaudRate));
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    /* (divider * 10) computing in case Oversampling mode is 16 Samples */
    divider = (uint32_t)((apbclock) / (USART_InitStruct->USART_BaudRate));
    tmpreg  = (uint32_t)((apbclock) % (USART_InitStruct->USART_BaudRate));
  }
  
  /* round the divider : if fractional part i greater than 0.5 increment divider */
  if (tmpreg >=  (USART_InitStruct->USART_BaudRate) / 2)
  {
    divider++;
  } 
  
  /* Implement the divider in case Oversampling mode is 8 Samples */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
  {
    /* get the LSB of divider and shift it to the right by 1 bit */
    tmpreg = (divider & (uint16_t)0x000F) >> 1;
    
    /* update the divider value */
    divider = (divider & (uint16_t)0xFFF0) | tmpreg;
  }
  
  /* Write to USART BRR */
  USARTx->BRR = (uint16_t)divider;
}

void USART_ITConfig(USART_TypeDef* USARTx, uint32_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0, itpos = 0, itmask = 0;
  uint32_t usartxbase = 0;

  usartxbase = (uint32_t)USARTx;
  
  /* Get the USART register index */
  usartreg = (((uint16_t)USART_IT) >> 0x08);
  
  /* Get the interrupt position */
  itpos = USART_IT & IT_MASK;
  itmask = (((uint32_t)0x01) << itpos);
  
  if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x04;
  }
  else if (usartreg == 0x03) /* The IT is in CR3 register */
  {
    usartxbase += 0x08;
  }
  else /* The IT is in CR1 register */
  {
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)usartxbase &= ~itmask;
  }
}

/*
eint13 check
*/
void mySYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
	#if 1
  uint32_t tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_EXTI_PORT_SOURCE(EXTI_PortSourceGPIOx));
  assert_param(IS_EXTI_PIN_SOURCE(EXTI_PinSourcex));
  
  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
	#else
	uint32_t tmp = 0x00;
	uint32_t offset = 0;
	uint32_t value = 0;

  /* Check the parameters */
  assert_param(IS_EXTI_PORT_SOURCE(EXTI_PortSourceGPIOx));
  assert_param(IS_EXTI_PIN_SOURCE(EXTI_PinSourcex));
  
  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
	offset = EXTI_PinSourcex >> 0x02;
	value =  (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
  SYSCFG->EXTICR[offset] &= ~tmp;
  SYSCFG->EXTICR[offset] |= value;
	#endif
}
void myEXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
  assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
  assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));
  assert_param(IS_FUNCTIONAL_STATE(EXTI_InitStruct->EXTI_LineCmd));

  tmp = (uint32_t)EXTI_BASE;

  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;

    tmp += EXTI_InitStruct->EXTI_Mode;

    *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;

    /* Select the trigger for the selected interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    {
      /* Rising Falling edge */
      EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
      EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      tmp = (uint32_t)EXTI_BASE;
      tmp += EXTI_InitStruct->EXTI_Trigger;

      *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    tmp += EXTI_InitStruct->EXTI_Mode;

    /* Disable the selected external lines */
    *(__IO uint32_t *) tmp &= ~EXTI_InitStruct->EXTI_Line;
  }
}

ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint32_t USART_IT)
{
  uint32_t bitpos = 0, itmask = 0, usartreg = 0;
//	uint32_t isr = 0x00;
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_GET_IT(USART_IT)); 
  
  /* Get the USART register index */
  usartreg = (((uint16_t)USART_IT) >> 0x08);
  /* Get the interrupt position */
  itmask = USART_IT & IT_MASK;
  itmask = (uint32_t)0x01 << itmask;
  
  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= USARTx->CR2;
  }
  else /* The IT  is in CR3 register */
  {
    itmask &= USARTx->CR3;
  }
  
  bitpos = USART_IT >> 0x10;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= USARTx->ISR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}


void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint32_t USART_IT)
{
  uint32_t bitpos = 0, itmask = 0;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT)); 
  
  bitpos = USART_IT >> 0x10;
  itmask = ((uint32_t)0x01 << (uint32_t)bitpos);
	itmask |= 0x08;
  USARTx->ICR = (uint32_t)itmask;
}

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
	int32_t retry = 20000;
  /* Check the parameters */
 // assert_param(IS_USART_ALL_PERIPH(USARTx));
//  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->TDR = (Data & (uint16_t)0x01FF);
	while(!(USARTx->ISR & (1<<6))) {
		retry--;
		if (retry < 1) {
			return;
		}
		//DelayMs(1);
	}
	
	//karl
}

static void app_uart_write(uint8_t *pbuf)
{
	int i;

	while(*pbuf) {
		USART_SendData(USART1, *pbuf);
		pbuf++;
	}
} 


/*
called by uart2_init
*/

static void USART1_dma_config()
{
	DMA_InitTypeDef DMA_InitStructure;
		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA1??5??
	DMA_DeInit(DMA1_Channel3);
	//????
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
	//????
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_uart1_dmabuf;
	//dma??????
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//??DMA??????????
	DMA_InitStructure.DMA_BufferSize = UART1_DMA_BUF;
	//??DMA???????,????
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//??DMA???????
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//??????
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//??????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//??DMA?????
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//??DMA?????
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	//??DMA?2?memory????????
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3,&DMA_InitStructure);
	//????5
	DMA_Cmd(DMA1_Channel3,ENABLE);
}

static void led_init()
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN | LED_RED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_BLUE_PIN_GROUP, &GPIO_InitStructure);
}

static void USART1_Init()
{

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configure clock GPIO, USARTs */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//Reset USART1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
  /* USART1 Pins configuration  ***********************************************/  
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);   
	
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//init_doorlock_gpio();
	USART1_dma_config();
	/* 8xUSARTs configuration --------------------------------------------------*/
  /* 8xUSARTs  configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART1->ICR |= 1<<4; //?????IDLE??,??????IDLE??
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);


	/*ENABLE USART1*/
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1,ENABLE);	
	
	/* USART1 IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler(void)
{
	uint32_t recvLen = 0;
	volatile uint8_t temp;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART1->ICR |= 1<<4; //????
		DMA_Cmd(DMA1_Channel3,DISABLE);
		//Len = UART1_DMA_BUF - DMA_GetCurrDataCounter(DMA1_Channel5);
		//if (Len < MAX_UART_BUFFER_SIZE) {
		//	memcpy(g_uart2_buf, g_uart1_dmabuf, Len);
		//	g_uart2_index = Len;
		//} else {
		//	memcpy(g_uart2_buf, g_uart1_dmabuf, MAX_UART_BUFFER_SIZE);
		//	g_uart2_index = MAX_UART_BUFFER_SIZE;
		//}
	
		recvLen = UART1_DMA_BUF - DMA_GetCurrDataCounter(DMA1_Channel3);
		if (RET_SUCCESS != app_fifo_write(&m_rx_fifo, g_uart1_dmabuf, &recvLen)) {
		}
		
		
		DMA_SetCurrDataCounter(DMA1_Channel3,UART1_DMA_BUF);
		//??DMA
		DMA_Cmd(DMA1_Channel3,ENABLE);
		//karl_uart_event_handle();
	}
	USART_ClearITPendingBit(USART1, USART_IT_ORE);
}


int my_itoa(uint32_t n,uint8_t *s)
{
	int i,j,k;
	char temp[8];
	
	i=0;
	do{
		temp[i++]=n%10+'0'; 
	}while((n/=10)>0);

	k = 0;
	for(j=i-1;j>=0;j--)     
		s[k++] = temp[j];
	
	return k;
}

int my_atoi(uint8_t *ptr)
{
	int number  = 0;
	
	while(*ptr) {
		if ((*ptr >= '0') && (*ptr <= '9')) {
			number = number * 10;
			number += *ptr - '0';
		} else {
			return number;
		}
		ptr++;
	}
}


#define KR_KEY_RELOAD    ((uint16_t)0xAAAA)
void IWDG_ReloadCounter(void)
{
  IWDG->KR = KR_KEY_RELOAD;
}

void save_firmware_flash(uint32_t offset, uint8_t *buf, uint32_t count)
{
//#pragma pack (4)
	//uint8_t savebuf[64];
	//#pragma pack()
	
	//memcpy(savebuf, buf, count);
	/*
		FLASH_Unlock();
		FLASH_ErasePage(USER_NVRAM_DATA_ADDRESS);
		FLASH_ProgramBuf(USER_NVRAM_DATA_ADDRESS, NULL,  0);
		FLASH_Lock();
	*/
	if ((offset & 0x7ff) == 0) {
		IWDG_ReloadCounter();
		FLASH_ErasePage(APPLICATION_FLASH_START_ADDRESS + offset);
	}
	FLASH_ProgramBuf(APPLICATION_FLASH_START_ADDRESS + offset, buf, count);
}




#define LED_BLUE_CLR()  LED_BLUE_PIN_GROUP->BRR = LED_BLUE_PIN
#define LED_BLUE_SET() LED_BLUE_PIN_GROUP->BSRR = LED_BLUE_PIN

static void led_blink(void)
{
	static bool mledon = false;
	if (mledon) {
		LED_BLUE_CLR();
		mledon = false;
	} else {
		LED_BLUE_SET();
		mledon = true;
	}
}
void DelayMs(uint16_t time);
static void modem_init_and_power_on(void)
{
			
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	/* Enable SYSCFG clock */

	//PA4 POWER KEY
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOD, GPIO_Pin_2);	
	GPIOD->BSRR = GPIO_Pin_2;
	
	//PA5 POWER
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	

	//GPIO_SetBits(GPIOF, GPIO_Pin_0);							//enable gsm power
	//GPIOF->BSRR = GPIO_Pin_0;
	GPIOF->BRR = GPIO_Pin_0;			//disable gsm power
	DelayMs(1000);
	GPIOF->BSRR = GPIO_Pin_0;			//enable gsm power
	DelayMs(1000);
	
	GPIOD->BSRR = GPIO_Pin_2;
	DelayMs(800);
	GPIOD->BRR = GPIO_Pin_2;
	DelayMs(800);
}

#define LINE_SIZE_MAX 128
uint8_t mCurrentRecvLine[LINE_SIZE_MAX];
static bool strStartWith(uint8_t *src, uint8_t *dst)
{
	while(*src && *dst) {
		if (*src != *dst) {
			return false;
		}
		src++;
		dst++;
	}
	if (*dst == '\0') return true;
	return false;
}

#define NORMAL_AT_COMMAND_WAIT_TIME 4000000
#define NET_AT_COMMAND_WAIT_TIME 9000000
static bool wait_at_resp(uint8_t *pwait, uint32_t retry)
{
	uint32_t i;
	//uint32_t retry = 4000000;
	uint32_t index = 0;
	
	for (i = 0; i < retry; i++) {
		if (fifo_length(&m_rx_fifo)>0) {
			app_fifo_get(&m_rx_fifo, &mCurrentRecvLine[index]);
			if (mCurrentRecvLine[index] == '\n') {
				mCurrentRecvLine[index] = '\0';
				if (strStartWith(mCurrentRecvLine, pwait)) {
					return true;
				} else {
					index = 0;
					continue;
				}
			}
			index++;
		}
	}
	return false;
}

static bool send_normal_at_command(const char *cmd, const char *waitresp,uint32_t retry)
{
	uint32_t i;
	uint32_t index = 0;
	
	while (fifo_length(&m_rx_fifo) > 0) {
		app_fifo_get(&m_rx_fifo, mCurrentRecvLine); //dummy all prefer data
	}
	
	app_uart_write((uint8_t*)cmd);
	return wait_at_resp((uint8_t*)waitresp, retry);
}

static int get_firmware_size(void)
{
	uint32_t i;
	uint32_t index = 0;
	int size = 0;
	
	while (fifo_length(&m_rx_fifo) > 0) {
		app_fifo_get(&m_rx_fifo, mCurrentRecvLine); //dummy all prefer data
	}
	
	app_uart_write((uint8_t*)"AT+FTPSIZE=\"m12path_091_m8321.bin\"\r\n");
	if (!wait_at_resp((uint8_t*)"+FTPSIZE:", NET_AT_COMMAND_WAIT_TIME)) {
		return 0;
	}
	size = my_atoi(&mCurrentRecvLine[9]);
	if (!wait_at_resp((uint8_t*)"OK\r", NORMAL_AT_COMMAND_WAIT_TIME)) {
		return 0;
	}
	return size;
}

#define FTP_GET_CMD_HEADER "AT+FTPGET=\"m12path_091_m8321.bin\","
#define  MAX_RECV_DATA_AT_SIZE  512
static int pack_ftp_get_cmd(uint8_t *pcmd, uint32_t curindex)
{
	uint8_t nbuf[8];
	int index = 0;
	int size = sizeof(FTP_GET_CMD_HEADER) -1;
	int recallsize = MAX_RECV_DATA_AT_SIZE;
	
	if (recallsize > (mFirmwareSize - curindex)) {
		recallsize = mFirmwareSize - curindex;
	}
	my_memcpy(pcmd, (uint8_t*)FTP_GET_CMD_HEADER, size);
	index = size;
	size = my_itoa(curindex, nbuf);
	my_memcpy(&pcmd[index], nbuf, size);
	index += size;
	pcmd[index++] = ',';
	
	size = my_itoa(recallsize, nbuf);
	my_memcpy(&pcmd[index], nbuf, size);
	index += size;

	pcmd[index++] = '\r';
	pcmd[index++] = '\n';
	pcmd[index] = '\0';
	
	return recallsize;
}


static bool check_modem(void)
{
	int i;
	
	for (i = 0; i < 10; i++) {
		if (send_normal_at_command("AT\r\n", "OK\r", NORMAL_AT_COMMAND_WAIT_TIME)) {
			break;
		}
		DelayMs(500);
	}
	if (i >= 10) {
		return false;
	}
	send_normal_at_command("ATE1\r\n", "OK\r", NORMAL_AT_COMMAND_WAIT_TIME);
	send_normal_at_command("ATE1\r\n", "OK\r", NORMAL_AT_COMMAND_WAIT_TIME);
	send_normal_at_command("AT+ZISSENDAUTO=0\r\n", "OK\r", NORMAL_AT_COMMAND_WAIT_TIME);
	for (i = 0; i < 10; i++) {
		if (send_normal_at_command("AT+S32K=0\r\n", "OK\r", NORMAL_AT_COMMAND_WAIT_TIME)) {
			break;
		}
		DelayMs(100);
	}
	if (i>=10) {
		return false;
	}
	return true;
}

static bool ftp_open(void)
{
	int i = 0;
	
	for (i = 0; i < 100; i++) {
		if (send_normal_at_command("AT+FTPOPEN=0,\"tuling-iot.com\",\"test\",\"test\",1,20,0\r\n", "OK\r", NET_AT_COMMAND_WAIT_TIME)) {
			break;
		}
		DelayMs(300);
	}
	if (i >= 100) return false;
	mFirmwareSize = get_firmware_size();
	if (mFirmwareSize <= 1024) {
		return false;
	}
	/*
	if (send_normal_at_command("AT+FTPGET=\"m12path_091_m8321.bin\",0,16\r\n", "OK\r", NET_AT_COMMAND_WAIT_TIME)) {
	}
	*/
	return true;
}

static int mHttpCurrentRecvIndex = 0;
#pragma pack (4)
static uint8_t mHttpOnePageBuf[2048];		
#pragma pack()


static int handle_http_read(void)
{
	int ret;
	int currecv = 0;
	uint32_t cmdretry = 0;
	uint32_t retry = 100000;
	uint32_t i, j;
	uint32_t index = 0;
	uint8_t bufcmd[64];
	
	while(1) {
		currecv = pack_ftp_get_cmd(bufcmd, mHttpCurrentRecvIndex);
		//ret = snprintf((char*)bufcmd, 32, "AT+HTTPREAD=%d,%d\r\n", mHttpCurrentRecvIndex, MAX_RECV_DATA_AT_SIZE);
		//pack_http_read_cmd(bufcmd, mHttpCurrentRecvIndex);
	//2.seek the position
		for (cmdretry = 0; cmdretry < 3; cmdretry++) {
			while (fifo_length(&m_rx_fifo) > 0) {
				app_fifo_get(&m_rx_fifo, mCurrentRecvLine); //dummy all prefer data
			}
			
			app_uart_write(bufcmd);		//send command
		
			if (!wait_at_resp((uint8_t*)"AT+FTPGET=\"m12path_091_m8321.bin\",", NET_AT_COMMAND_WAIT_TIME*2)) {
				continue;
			}
			break;
		}
		if (cmdretry >= 3) 
			return -1;
		
		if (currecv < 0) {
			return -1;
		}
		//read data
		for (i = 0; i < currecv; i++) {
			for (j = 0; j < NET_AT_COMMAND_WAIT_TIME;j++) {
				if (fifo_length(&m_rx_fifo) > 0) {
					break;
				}
			}
			if (fifo_length(&m_rx_fifo)) {
				app_fifo_get(&m_rx_fifo, &mHttpOnePageBuf[i]);
			} else {
				return -1;
			}
		}
		IWDG_ReloadCounter();
		if (!wait_at_resp((uint8_t*)"OK\r", NORMAL_AT_COMMAND_WAIT_TIME)) {
			return -1;
		}
		save_firmware_flash(mHttpCurrentRecvIndex, mHttpOnePageBuf, currecv);
		mHttpCurrentRecvIndex += currecv;
		//led_blink();
		if (currecv < MAX_RECV_DATA_AT_SIZE) {
			return 0;
		}
	}
	return 1;
}

void update_param_flag(bool success)
{
	UserNvdata *pfnv = (UserNvdata *)USER_NVRAM_DATA_ADDRESS;
	
	#pragma pack (4)
	UserNvdata tmpNv;
	#pragma pack()
	
	memcpy(&tmpNv, pfnv, sizeof(UserNvdata));
	tmpNv.FlashFlag = 0x3344;
	/*
		FLASH_Unlock();
		FLASH_ErasePage(USER_NVRAM_DATA_ADDRESS);
		FLASH_ProgramBuf(USER_NVRAM_DATA_ADDRESS, NULL,  0);
		FLASH_Lock();
	*/
	IWDG_ReloadCounter();
	FLASH_ErasePage(USER_FLASH_START_ADDRESS);
	FLASH_ProgramBuf(USER_FLASH_START_ADDRESS, (uint8_t*)&tmpNv, sizeof(UserNvdata));
}

volatile bool success = true;
int main_upgrade_function(void)
{
			
	if (RET_SUCCESS != app_fifo_init(&m_rx_fifo, g_uart2_user_buf, sizeof(g_uart2_user_buf))) {
		while(1);	
	}
	
	modem_init_and_power_on();
	USART1_Init();
	
	if(!check_modem()) {
		return -1;
	}
	
	if (!ftp_open()) {
		return -1;
	}
	DelayMs(8000);//wait for fota command 8 second
	FLASH_Unlock();
	if(handle_http_read() == 0) {
		//upgrade sucess
		update_param_flag(true);
	}
	FLASH_Lock();
}


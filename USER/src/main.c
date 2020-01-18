/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "userflash.h"
void my_memcpy(uint8_t *dst, uint8_t *src, int len);
#define PROGRAM_START_ADDR  0x0800D000
#define PROGRAM_RAM_VECTOR_ADDR 0x20000000
#define VECTOR_SIZE 0xB4

void DelayMs(uint16_t time)
{    
   uint32_t i=0;  
   while(time--)
   {
      i=12000;  //????
      while(i--) ;    
   }
}

void SYSCFG_MemoryRemapConfig(uint32_t SYSCFG_MemoryRemap)
{
  uint32_t tmpctrl = 0;

  /* Check the parameter */
  assert_param(IS_SYSCFG_MEMORY_REMAP(SYSCFG_MemoryRemap));

  /* Get CFGR1 register value */
  tmpctrl = SYSCFG->CFGR1;

  /* Clear MEM_MODE bits */
  tmpctrl &= (uint32_t) (~SYSCFG_CFGR1_MEM_MODE);

  /* Set the new MEM_MODE bits value */
  tmpctrl |= (uint32_t) SYSCFG_MemoryRemap;

  /* Set CFGR1 register with the new memory remap configuration */
  SYSCFG->CFGR1 = tmpctrl;
}

uint32_t MySysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
  //NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

volatile uint32_t mSysTickCount = 0;
void SysTick_Handler(void)
{
	mSysTickCount++;
}
#if 0
uint32_t idAddr[]={0x1FFFF7AC,  /*STM32F0??ID????*/
                0x1FFFF7E8,  /*STM32F1??ID????*/
                0x1FFF7A10,  /*STM32F2??ID????*/
                0x1FFFF7AC,  /*STM32F3??ID????*/
                0x1FFF7A10,  /*STM32F4??ID????*/
                0x1FF0F420,  /*STM32F7??ID????*/
                0x1FF80050,  /*STM32L0??ID????*/
                0x1FF80050,  /*STM32L1??ID????*/
                0x1FFF7590,  /*STM32L4??ID????*/
                0x1FF0F420}; /*STM32H7??ID????*/
#endif

#define ID_ADDR_F030 0x1FFFF7AC
uint32_t GetSTM32MCUID_F030_Priv()
{
	return *(uint32_t*)(ID_ADDR_F030) +
			*(uint32_t*)(ID_ADDR_F030+4)	+
			*(uint32_t*)(ID_ADDR_F030+8);
}

__asm void MSR_MSP(uint32_t	addr) 
{
    MSR MSP, r0 //set Main Stack value
    BX r14
}

typedef  void (*pFunction)(void);

void jump_to_normal_app()
{
	pFunction Jump_To_Application;
	uint32_t JumpAddress;
	
	NVIC->ICER[0] = 0xffffffff;		//disalbe all interrupt
	JumpAddress = *(volatile uint32_t*)(APPLICATION_FLASH_START_ADDRESS+4);
	Jump_To_Application = (pFunction)JumpAddress;
	//IWDG_SetReload(4000);
	MSR_MSP(APPLICATION_FLASH_START_ADDRESS);
	Jump_To_Application();
	
	//test_led();
}
volatile uint32_t mLoraMyAddr;
volatile uint32_t mLoraGatewayAddr;
void main_upgrade_function(void);
int main()
{
	UserNvdata *pfnv = (UserNvdata *)USER_NVRAM_DATA_ADDRESS;
	mLoraMyAddr = GetSTM32MCUID_F030_Priv();
	
	mLoraGatewayAddr = pfnv->MASTERADDR;
	
	//MySysTick_Config(SystemCoreClock/1000);
	
	if (pfnv->FlashFlag == 0x1122) { //need upgrade
		main_upgrade_function();
	} else {
		jump_to_normal_app();
	}
	__NVIC_SystemReset();
	__NVIC_SystemReset();
	
	while(1) {
		DelayMs(1000);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

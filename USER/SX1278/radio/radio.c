/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       radio.c
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.0 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */
 
//#include <stdint.h>
#include "stm32f0xx.h"
#include "contiki.h"
#include "radio.h"

#include "sx1276.h"

enum {
	PROCESS_ACTION_SEND_LORA_DATA = 1,
	PROCESS_ACTION_SEND_DONE
};

#if 0
tRadioDriver* RadioDriverInit( void )
{
#if defined( USE_SX1232_RADIO )
    RadioDriver.Init = SX1232Init;
    RadioDriver.Reset = SX1232Reset;
    RadioDriver.StartRx = SX1232StartRx;
    RadioDriver.GetRxPacket = SX1232GetRxPacket;
    RadioDriver.SetTxPacket = SX1232SetTxPacket;
    RadioDriver.Process = SX1232Process;
#elif defined( USE_SX1272_RADIO )
    RadioDriver.Init = SX1272Init;
    RadioDriver.Reset = SX1272Reset;
    RadioDriver.StartRx = SX1272StartRx;
    RadioDriver.GetRxPacket = SX1272GetRxPacket;
    RadioDriver.SetTxPacket = SX1272SetTxPacket;
    RadioDriver.Process = SX1272Process;
#elif defined( USE_SX1276_RADIO )
    RadioDriver.Init = SX1276Init;
    RadioDriver.Reset = SX1276Reset;
    RadioDriver.StartRx = SX1276StartRx;
    RadioDriver.GetRxPacket = SX1276GetRxPacket;
    RadioDriver.SetTxPacket = SX1276SetTxPacket;
    RadioDriver.Process = SX1276Process;
#else
    #error "Missing define: USE_XXXXXX_RADIO (ie. USE_SX1272_RADIO)"
#endif    

    return &RadioDriver;
}
#endif

PROCESS(process_sx1278, "sx1278 process");

PROCESS_THREAD(process_sx1278, ev, data)
{

  PROCESS_BEGIN();
	
	SX1276Init();

  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				//SX1276OnDio0IrqByKarl();
				break;
			
			case PROCESS_EVENT_TIMER:
				break;
			
			case PROCESS_ACTION_SEND_LORA_DATA:
				break;
			
			case PROCESS_ACTION_SEND_DONE:
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


void EXTI4_15_IRQHandler(void)
{
	uint8_t pinVal = 0;
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		process_poll(&process_sx1278);
		EXTI_ClearITPendingBit(EXTI_Line6);
  } else {
		EXTI_ClearITPendingBit(EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15);
	}
}




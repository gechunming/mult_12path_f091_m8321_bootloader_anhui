#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "contiki.h"
#include "stm32f0xx.h"
#include "lorawan.h"
#include "radio.h"
#include "sx1276.h"

typedef enum _LORAWAN_STATUS {
	LORAWAN_IDLE,
	LORAWAN_SENDING,
	LORAWAN_RECEIVING,
	LORAWAN_WAITACK
} LORAWAN_STATUS;

typedef enum _LORAWAN_EVENT {
	LORAWAN_EVENT_TIMEOUT,
	LORAWAN_EVENT_TXDONE,
	LORAWAN_EVENT_RXDONE,
	LORAWAN_EVENT_TXDATAAVAILABLE,
} LORAWAN_EVENT;

#define MAX_RETRY_SEND_TIMES 2
struct LoraDeviceInfo {
	DeviceClass_t devclass;
	LORAWAN_STATUS status;
	
	/* used for adr */
	uint8_t datarateindex;
	uint8_t datarate[MAX_RETRY_SEND_TIMES];
	
	struct etimer etimer;
	
	/* data next send */
	uint8_t applen;
	uint8_t appdata[256];
	Mcps_t mcps;				     /*current msg type */
	bool  txavail;
};

struct LoraDeviceInfo gLoraDev;
void print_string(char *str);
static void lorawan_status_machine(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event);

PROCESS(process_lorawan, "user key process");

/*
for user operation
*/
void handle_lora_tx_status(bool sucess);

bool lora_send_user_data(uint8_t *ubuf, uint8_t len, bool confirmed)
{

	if ((gLoraDev.status == LORAWAN_IDLE) ||
		 (gLoraDev.status == LORAWAN_RECEIVING)) {
			 
			 PROCESS_CONTEXT_BEGIN(&process_lorawan);
			 
			 gLoraDev.applen = len;
			 memcpy(&gLoraDev.appdata, ubuf, len);
			 gLoraDev.mcps = confirmed? MCPS_CONFIRMED: MCPS_UNCONFIRMED;
			 gLoraDev.txavail = true;
			 lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDATAAVAILABLE);
			 gLoraDev.datarateindex = 0;
			 
			 PROCESS_CONTEXT_END(&process_lorawan);
			 print_string("u tx\r\n");
			return true; 
		 } else {
			 print_string("u tx b\r\n");
			 return false;
		 }
}
void send_kongkai_control_command(uint8_t path, uint8_t value);
void handle_lorawan_rx(uint8_t *ubuf, uint8_t len)
{
	/*
	if (*ubuf == 0x23) {
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
	} else {
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	}
	*/
	if (len < 3) return;
	//cmd, index, value
	switch(ubuf[0]) {
		case 0x01: //for set value command
			send_kongkai_control_command(ubuf[1], ubuf[2]);
			break;
			
		default:
			break;
	}
}

static void lorawan_init(void)
{
	uint8_t i;
	memset(&gLoraDev, 0x00, sizeof(gLoraDev));
	for (i = 0; i < MAX_RETRY_SEND_TIMES; i++) {
		gLoraDev.datarate[i] = 7;
	}
	gLoraDev.txavail = false;
	gLoraDev.mcps = MCPS_UNCONFIRMED;
	gLoraDev.status = LORAWAN_IDLE;
	gLoraDev.devclass = CLASS_C;
	SX1276Init(0x00);
	SX1276SetStby();
	SX1276SetChannel(471700000);
	SX1276SetPublicNetwork(true);
	SX1276SetRxConfig(MODEM_LORA, 0, gLoraDev.datarate[0], 2, 0, 0xff, 0x3ff, false, 0, true, false, 4, false, true);
}

static void set_lora_receve_mode(struct LoraDeviceInfo *pDev)
{
		SX1276SetStby();
		SX1276SetRxConfig(MODEM_LORA, 0,  pDev->datarate[0], 2, 0, 8, 0x3ff, false, 0, true, false, 4, false, true);
		SX1276SetRx(0x00);
}

static void send_lora_data_internal(struct LoraDeviceInfo *pDev)
{
		PrepareFrame(pDev->appdata, pDev->applen, pDev->mcps);		
		SX1276SetStby();
		SX1276SetTxConfig(MODEM_LORA, 20, 0, 0, pDev->datarate[0], 2, 8, false, true, false, 4, false, 0x00);
		ScheduleTx();  //will goto 
}

void set_ack_report()
{
	gLoraDev.txavail = true;
	lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDATAAVAILABLE);
}

static void lora_status_set(struct LoraDeviceInfo *pdev, LORAWAN_STATUS status)
{
	pdev->status = status;
}
/*
  idle ×´Ì¬´¦Àí
*/
static void lora_idle_status_func(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event) 
{
	switch(event) {
		default:
			if (pDev->txavail) {
				send_lora_data_internal(pDev);
				pDev->txavail = false;
				lora_status_set(pDev, LORAWAN_SENDING);
				etimer_set(&pDev->etimer, CLOCK_SECOND);			//use 1 seconds for sending done
				break;
			}
			if (pDev->devclass == CLASS_C) {
				//tranmit to lorareceive mode
				set_lora_receve_mode(pDev);
				lora_status_set(pDev, LORAWAN_RECEIVING);
			}
			break;
	}
	return;
}

static void lora_sending_status_func(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event)
{
	switch(event) {
		case LORAWAN_EVENT_TIMEOUT: //send timeout then reset lora, and enter into ilde mode
			print_string("sending tm\r\n");
			lora_status_set(pDev, LORAWAN_IDLE);
			pDev->mcps = MCPS_UNCONFIRMED;
			pDev->applen = 0;
			handle_lora_tx_status(false);
			lora_idle_status_func(pDev, LORAWAN_EVENT_TIMEOUT);
			break;
			
		case LORAWAN_EVENT_TXDONE:
			print_string("tx done\r\n");
			set_lora_receve_mode(pDev);
			if (pDev->mcps == MCPS_UNCONFIRMED) {
				lora_status_set(pDev, LORAWAN_RECEIVING);
				pDev->mcps = MCPS_UNCONFIRMED;
				pDev->applen = 0;
				handle_lora_tx_status(true);
				lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDATAAVAILABLE);
				etimer_set(&pDev->etimer, CLOCK_SECOND * 10);	 //for type A device receive window
			} else {
				lora_status_set(pDev, LORAWAN_WAITACK);
				etimer_set(&pDev->etimer, CLOCK_SECOND);			
			}
			break;
			
		default:
			break;
	}
}

static void lora_waitack_status_func(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event)
{
	switch(event) {
		case LORAWAN_EVENT_TIMEOUT:
			print_string("wait ack tm\r\n");
			if (pDev->datarateindex < (MAX_RETRY_SEND_TIMES-1)) {
				pDev->datarateindex++;
				send_lora_data_internal(pDev);
				etimer_set(&pDev->etimer, CLOCK_SECOND);		
			} else {
				lora_status_set(pDev, LORAWAN_IDLE);
				pDev->mcps = MCPS_UNCONFIRMED;
				pDev->applen = 0;
				lora_idle_status_func(pDev, LORAWAN_EVENT_TIMEOUT);
				handle_lora_tx_status(false);
				lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDATAAVAILABLE);
			}
			break;
		
		case LORAWAN_EVENT_TXDONE:
			set_lora_receve_mode(pDev);
			etimer_set(&pDev->etimer, CLOCK_SECOND);
			break;
			
		case LORAWAN_EVENT_RXDONE:
			print_string("waitack getted\r\n");
			lora_status_set(pDev, LORAWAN_RECEIVING);
			set_lora_receve_mode(pDev);
			pDev->mcps = MCPS_UNCONFIRMED;
			pDev->applen = 0;
			handle_lora_tx_status(true);
			lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDATAAVAILABLE);
			etimer_set(&pDev->etimer, CLOCK_SECOND * 10); //for type B device
			break;
			
		default:
			break;
	}
}

static void lora_receving_status_func(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event)
{
	switch(event) {
		case LORAWAN_EVENT_TIMEOUT:
			print_string("receiving tm\r\n");
			if (pDev->devclass == CLASS_A) {
				lora_status_set(pDev, LORAWAN_IDLE);
				lora_idle_status_func(pDev, LORAWAN_EVENT_TIMEOUT);
			}
			break;
			
		case LORAWAN_EVENT_TXDATAAVAILABLE:
			if (pDev->txavail) {
				send_lora_data_internal(pDev);
				pDev->txavail = false;
				lora_status_set(pDev, LORAWAN_SENDING);
				etimer_set(&pDev->etimer, CLOCK_SECOND);			//use 1 seconds for sending done
				break;
			}
			break;
			
		default:
			if (pDev->txavail) {
				send_lora_data_internal(pDev);
				pDev->txavail = false;
				lora_status_set(pDev, LORAWAN_SENDING);
				break;
			}
			break;
	}	
}

static void lorawan_status_machine(struct LoraDeviceInfo *pDev, LORAWAN_EVENT event)
{
	switch(pDev->status) {	
		case LORAWAN_SENDING:
			lora_sending_status_func(pDev, event);
			break;
			
		case LORAWAN_RECEIVING:
			lora_receving_status_func(pDev, event);
			break;
			
		case LORAWAN_WAITACK:
			lora_waitack_status_func(pDev, event);
			break;
			
		case LORAWAN_IDLE:
		default:
			lora_idle_status_func(pDev, event);
			break;
	}
}

/*
lorawan callbacks
*/
void notify_last_snr_value(int8_t snr)
{
	return;
}

void radio_tx_done_callback(void)
{
	lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TXDONE);
}

void handle_lorawandata_rxdone(uint8_t *userdata, uint32_t len)
{
	lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_RXDONE);
	if (len != 0) 
		handle_lorawan_rx(userdata, len);
}


PROCESS_THREAD(process_lorawan, ev, data)
{
  PROCESS_BEGIN();
	
	lorawan_init();
	lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TIMEOUT);
	
	while(1) {
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL:
				SX1276OnDio0IrqByKarl();
				break;
				
			case PROCESS_EVENT_TIMER:
				lorawan_status_machine(&gLoraDev, LORAWAN_EVENT_TIMEOUT);
				break;
				
			default:
				break;
		}
  }

  PROCESS_END();
}


void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)) {
			process_poll(&process_lorawan);
		} 
    EXTI_ClearITPendingBit(EXTI_Line0);
  } else {
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

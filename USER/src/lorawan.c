#include <stdint.h>
#include <stdbool.h>

#include "radio.h"
#include "lorawan.h"

#define LORA_ADDR 0x44556677

/*
lora raw package buffer
*/
#define LORAMAC_PHY_MAXPAYLOAD 255
static uint8_t LoRaMacBuffer[LORAMAC_PHY_MAXPAYLOAD];
static uint16_t LoRaMacBufferPktLen = 0;
//static bool LoRaMacNeedConfirmed = false;

#define LORA_MAC_COMMAND_MAX_LENGTH 128
static bool MacCommandsInNextTx = false; //记录下次是否需要发送mac command
static uint8_t MacCommandsBufferIndex = 0;
static uint8_t MacCommandsBuffer[LORA_MAC_COMMAND_MAX_LENGTH];

static bool SrvAckRequested = false;

static uint16_t UpLinkCounter = 0x01;

extern bool NextTx;

static uint32_t caculateMIC(uint8_t *buf, int len)
{
	int i;
	uint8_t checksum = 0x00;
	
	for (i = 0; i < len; i++) {
		checksum += buf[i];
	}
	return (uint32_t)checksum;
}
/*
package user data to one lora frame
*/
void set_ack_report();
LoRaMacStatus_t PrepareFrame(uint8_t *appData, uint16_t appLen, Mcps_t mcps)
{
	LoRaMacHeader_t macheader;
	LoRaMacFrameCtrl_t framectrl;
	uint32_t macmic = 0x00;
	uint8_t pktHeaderLen = 0x00;
	uint8_t i;
	
	macheader.Value = 0x00;
	macheader.Bits.MType = (mcps == MCPS_CONFIRMED )?FRAME_TYPE_DATA_CONFIRMED_UP:FRAME_TYPE_DATA_UNCONFIRMED_UP;
	
	LoRaMacBuffer[pktHeaderLen++] = macheader.Value;
	LoRaMacBuffer[pktHeaderLen++] = (LORA_ADDR) & 0xff;
	LoRaMacBuffer[pktHeaderLen++] = (LORA_ADDR>>8) & 0xff;
	LoRaMacBuffer[pktHeaderLen++] = (LORA_ADDR>>16) & 0xff;
	LoRaMacBuffer[pktHeaderLen++] = ((LORA_ADDR>>24) & 0xff);
	
	framectrl.Value = 0x00;
	framectrl.Bits.Ack = 1;			/* every package has ack bit */
	framectrl.Bits.FPending = 1;
	LoRaMacBuffer[pktHeaderLen++] = framectrl.Value;
	
	LoRaMacBuffer[pktHeaderLen++] = UpLinkCounter & 0xff;
	LoRaMacBuffer[pktHeaderLen++] = (UpLinkCounter >> 8) & 0xff;
	
	if (MacCommandsInNextTx) {
		framectrl.Bits.FOptsLen = MacCommandsBufferIndex;
		LoRaMacBuffer[5] = framectrl.Value; //update framectrl field
		
		for (i = 0; i < MacCommandsBufferIndex; i++) {
			LoRaMacBuffer[pktHeaderLen++] = MacCommandsBuffer[i];
		}
		MacCommandsInNextTx = false;
	}
	
	//fport
	LoRaMacBuffer[pktHeaderLen++] = 0x01;
	//copy payload
	for (i = 0; i < appLen; i++) {
		LoRaMacBuffer[pktHeaderLen++] = appData[i];
	}
	
	macmic = caculateMIC(LoRaMacBuffer, pktHeaderLen);
	LoRaMacBuffer[pktHeaderLen++] = 0x00;
	LoRaMacBuffer[pktHeaderLen++] = 0x00;
	LoRaMacBuffer[pktHeaderLen++] = 0x00;
	LoRaMacBuffer[pktHeaderLen++] = macmic & 0xff;
	LoRaMacBufferPktLen = pktHeaderLen;
	
	return LORAMAC_STATUS_OK;
}

int ScheduleTx(void)
{
	char buf[12];
	sprintf(buf, "send %d\r\n", LoRaMacBufferPktLen);
	print_string(buf);
	SX1276Send(LoRaMacBuffer, LoRaMacBufferPktLen);
	return 0;
}
void handle_lorawandata_rxdone(uint8_t *userdata, uint32_t len);
extern bool g_AppDataConfirm;

void set_ack_report();
static int handle_data_up(uint8_t *buf, int len, bool confirmed)
{
	uint32_t devaddr = 0x00;
	LoRaMacFrameCtrl_t fctrl;
	uint32_t payloadlen = 0x00;
	uint8_t fport = 0x00;
	uint32_t macmic = 0x00;
		uint32_t maccacu;

if (len < LORA_MAC_FRMPAYLOAD_OVERHEAD) {
		//printf("LoraWan:not enough payload data for lorawan\n");
		return -1;
	}


	macmic = caculateMIC(buf, len - 4);
	maccacu = (buf[len-4] << 24) | (buf[len-3] << 16) | (buf[len-2] << 8) | buf[len-1];
	if (macmic != maccacu) {
		//printf("lorawan macmic check failed %08x:%08x\n", macmic, maccacu);
		return -1;
	}
	
	//got one data up package
	devaddr = (buf[4] << 24) + (buf[3] << 16) + (buf[2] << 8) + (buf[1]);

	if (devaddr != LORA_ADDR) {
		//printf("lorawan not this device  %08x:%08x\n", devaddr, LORA_ADDR);
		return -1;
	}

	fctrl.Value = buf[5];
	if (fctrl.Bits.Ack) {
		//printf("Got command ack\n");
	//	g_AppDataConfirm = MCPS_UNCONFIRMED;
		//g_McpsConfirm.AckReceived = true;
		//every data package will be this ack,no need ack bit
	}

	payloadlen = len - LORA_MAC_FRMPAYLOAD_OVERHEAD - fctrl.Bits.FOptsLen;
	if (payloadlen <= 0) payloadlen = 0;

	//parse for mac command
	if (payloadlen > 0) {
		fport = buf[8 + fctrl.Bits.FOptsLen];

		if ((fport == 0) && !fctrl.Bits.FOptsLen) {
			//printf("LoraWan:Mac Command can't both in fopts and payload\n");
			handle_lorawandata_rxdone((void*)0, 0);
			return -1;
		} else if (fport == 0) {
			//printf("LoraWan:Mac Command in payload then process\n");
			//process for mac command
			//dumpbarray(&bufbuf[9 + fctrl.Bits.FOptsLen], payloadlen);
			handle_lorawandata_rxdone((void*)0, 0);
		} else {
			//printf("LoraWan: payload data:\n");
			//dumpbarray(&buf[9 + fctrl.Bits.FOptsLen], payloadlen);
			handle_lorawandata_rxdone(&buf[9 + fctrl.Bits.FOptsLen], payloadlen);
		}
	} else {
		handle_lorawandata_rxdone((void*)0, 0);
	}

	if (fctrl.Bits.FOptsLen != 0) {
		//process for mac command, in Fopts Len	
		//printf("LoraMacCommand:\n");
		//dumpbarray(&bufbuf[8], fctrl.Bits.FOptsLen);
	}

	if (confirmed) {
    set_ack_report();
	}
	return 0;
}

int handle_download_raw_data(uint8_t *buf, int len)
{
		LoRaMacHeader_t macheader;

	macheader.Value = buf[0];

	switch(macheader.Bits.MType) {
		case FRAME_TYPE_JOIN_REQ:
			
			break;

		case FRAME_TYPE_JOIN_ACCEPT:
			//no MIC
			//printf("LoraWan: Join accept\n");
			break;

		case FRAME_TYPE_DATA_UNCONFIRMED_UP:
			//printf("LoraWan: data unconfirmed up\n");
			break;

		case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
			//no this message for gateway
			//printf("LoraWan: data unconfirmed down\n");
			return handle_data_up(buf, len, false);
//			break;

		case FRAME_TYPE_DATA_CONFIRMED_UP:
			//printf("LoraWan: data confirmed up\n");
			break;

		case FRAME_TYPE_DATA_CONFIRMED_DOWN:
			//no this message for gateway
			//printf("Lorawan:data confirmed down\n");
			return handle_data_up(buf, len, true);
		//	break;

		case FRAME_TYPE_PROPRIETARY:
			//printf("Lorawan:data proprietary\n");
			break;

		case FRAME_TYPE_RFU:
		default:
			//printf("Lorawan data undefined RFU\n");
			break;
	}
	return 0;
}




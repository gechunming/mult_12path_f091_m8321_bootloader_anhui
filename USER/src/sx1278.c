#include "sx1278.h"
#include "string.h"
#include "stdlib.h"
		
#include "contiki.h"


PROCESS(process_sx1278, "sx1278 process");

static struct etimer mEtimer;
void print_stat_stat(uint8_t state, uint8_t state2);
#define RX_BUFFER_SIZE 128
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];


//#include "reg51.hcd"
lpCtrlTypefunc_t lpTypefunc = { 0, 0, 0 };
unsigned char power_data[8] = { 0X80, 0X80, 0X80, 0X83, 0X86, 0x89, 0x8c, 0x8f };

unsigned char Frequency[3] = { 0x75, 0x80, 0x00 };//470Mhz
//unsigned char Frequency[3] = { 0x6c, 0x80, 0x00 };//430Mhz ????
unsigned char powerValue = 7;
unsigned char SpreadingFactor = 8;    //????7-12
unsigned char CodingRate = 2;        //1-4
unsigned char Bw_Frequency = 7;      //??6-9
unsigned char RF_EX0_STATUS;
unsigned char CRC_Value;
unsigned char SX1278_RLEN;
//unsigned char recv[512];
void print_string(char *str);
void SPI_Init1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;

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
	
	RF_CE_H;
}

static void EXTI0_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void DelayMs(uint32_t nTime);
/*
function:SX1278 restart
*/
static void  SX1276Reset(void) {
	RF_REST_L;
	DelayMs(200);
	RF_REST_H;
	DelayMs(500);
}

/*
function:control GPIO-EN
*/
static void  cmdSwitchEn(cmdEntype_t cmd) {
	switch (cmd) {
	case enOpen: {
		RF_CE_L;
	}
		break;
	case enClose: {
		RF_CE_H;
	}
		break;
	default:
		break;
	}
}


static void uartSendString(char *str)
{
}

static void  RF_SPI_MasterIO(unsigned char out) {
	unsigned char i;
	for (i = 0; i < 8; i++) {
		if (out & 0x80) /* check if MSB is high */
			RF_SDI_H;
		else
			RF_SDI_L; /* if not, set to low */

		RF_CKL_H; /* toggle clock high */
		out = (out << 1); /* shift 1 place for next bit */
		RF_CKL_L; /* toggle clock low */
	}
}

static unsigned char   RF_SPI_READ_BYTE() {
	unsigned char j;
	unsigned char i;
	j = 0;
	for (i = 0; i < 8; i++) {
		RF_CKL_H;
		j = (j << 1);			// shift 1 place to the left or shift in 0 //
		if ( SX1278_SDO)					// check to see if bit is high //
			j = j | 0x01; 					   // if high, make bit high //
											   // toggle clock high //
		RF_CKL_L; 							 // toggle clock low //
	}

	return j;								// toggle clock low //
}
static unsigned char  SX1276ReadBuffer(unsigned char addr) {
	unsigned char Value;
	cmdSwitchEn(enOpen); //NSS = 0;
	RF_SPI_MasterIO(addr & 0x7f);
	Value = RF_SPI_READ_BYTE();
	cmdSwitchEn(enClose); //NSS = 1;

	return Value;
}

/*
function:set sx1278 work mode ,here you can control the mode which is send or recieve 
parm: 
*/
static void  SX1276LoRaSetOpMode(RFMode_SET opMode) {
	unsigned char opModePrev;
	opModePrev = SX1276ReadBuffer(REG_LR_OPMODE);
	opModePrev &= 0xf8;
	opModePrev |= (unsigned char) opMode;
	SX1276WriteBuffer( REG_LR_OPMODE, opModePrev);
}



static void  SX1276WriteBuffer(unsigned char addr,
		unsigned char buffer) {
	cmdSwitchEn(enOpen); //NSS = 0;
	RF_SPI_MasterIO(addr | 0x80);
	RF_SPI_MasterIO(buffer);
	cmdSwitchEn(enClose); //NSS = 1;
}
  
static void  SX1276LoRaSetRFFrequency(void) {
	SX1276WriteBuffer( REG_LR_FRFMSB, Frequency[0]);//0x04???????????
	SX1276WriteBuffer( REG_LR_FRFMID, Frequency[1]);//0x07???????????
	SX1276WriteBuffer( REG_LR_FRFLSB, Frequency[2]);//0x00???????????
}

static void  SX1276LoRaSetNbTrigPeaks(unsigned char value) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer(0x31);
	RECVER_DAT = (RECVER_DAT & 0xF8) | value;
	SX1276WriteBuffer(0x31, RECVER_DAT);
}

static void  SX1276LoRaSetSpreadingFactor(unsigned char factor) {
	unsigned char RECVER_DAT;
	SX1276LoRaSetNbTrigPeaks(3);
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT);
}

static void  SX1276LoRaSetErrorCoding(unsigned char value) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK)
			| (value << 1);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
// LoRaSettings.ErrorCoding = value;
}

static void  SX1276LoRaSetSignalBandwidth(unsigned char bw) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
// LoRaSettings.SignalBw = bw;
}

static void  SX1276LoRaSetImplicitHeaderOn(BOOL enable) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
			| (enable);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
}

static void  SX1276LoRaSetPayloadLength(unsigned char value) {
	SX1276WriteBuffer( REG_LR_PAYLOADLENGTH, value);
}

static void  SX1276LoRaSetSymbTimeout(unsigned int value) {
	unsigned char RECVER_DAT[2];
	RECVER_DAT[0] = SX1276ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT[1] = SX1276ReadBuffer( REG_LR_SYMBTIMEOUTLSB);
	RECVER_DAT[0] = (RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
			| ((value >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
	RECVER_DAT[1] = value & 0xFF;
	SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
	SX1276WriteBuffer( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}

static void  SX1276LoRaSetMobileNode(BOOL enable) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG3);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK)
			| (enable << 3);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG3, RECVER_DAT);
}

static void  RF_RECEIVE(void) {
	SX1276LoRaSetOpMode(Stdby_mode);
	SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //??????
	SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
	SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0X00);
	SX1276LoRaSetOpMode(Receiver_mode);
}

static void  SX1276LoRaSetPacketCrcOn(BOOL enable) {
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK)
			| (enable << 2);
	SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT);
}

static void  SX1276LoRaFsk(Debugging_fsk_ook opMode) {
	unsigned char opModePrev;
	opModePrev = SX1276ReadBuffer(REG_LR_OPMODE);
	opModePrev &= 0x7F;
	opModePrev |= (unsigned char) opMode;
	SX1276WriteBuffer( REG_LR_OPMODE, opModePrev);
}


static void  SX1276LoRaSetRFPower(unsigned char power) {
	SX1276WriteBuffer( REG_LR_PADAC, 0x87);
	SX1276WriteBuffer( REG_LR_PACONFIG, power_data[power]);
}


/*
function :you must call it ,the function is to init the module.
*/
static void  SX1276LORA_INT(void) {
	SX1276LoRaSetOpMode(Sleep_mode);  //??????0x01
	SX1276LoRaFsk(LORA_mode);	      // ??????,??????????
	SX1276LoRaSetOpMode(Stdby_mode);   // ???????
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, GPIO_VARE_1);
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, GPIO_VARE_1);
	SX1276WriteBuffer( REG_LR_DIOMAPPING2, GPIO_VARE_2);
	SX1276LoRaSetRFFrequency();
	SX1276LoRaSetRFPower(powerValue);
	SX1276LoRaSetSpreadingFactor(SpreadingFactor);	 // ??????
	SX1276LoRaSetErrorCoding(CodingRate);		 //?????
	SX1276LoRaSetPacketCrcOn(true);			  //CRC ????
	SX1276LoRaSetSignalBandwidth(Bw_Frequency);	 //??????
	SX1276LoRaSetImplicitHeaderOn(false);		//????????
	SX1276LoRaSetPayloadLength(0xff);//0x22 timeout??
	SX1276LoRaSetSymbTimeout(0x3FF);
	SX1276LoRaSetMobileNode(true); 			 // ??????
	RF_RECEIVE();
	uartSendString("\r\ninit finish\r\n");

}

/*
function :if you want to send data,you can call it 
RF_TRAN_P:data
ASM_i:the length of the data
*/
static void  FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P,
		unsigned char LEN) {
	unsigned char ASM_i;
//   lpTypefunc.paSwitchCmdfunc(txOpen);
	SX1276LoRaSetOpMode(Stdby_mode);
	SX1276WriteBuffer( REG_LR_HOPPERIOD, 0);	//??????
	SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_TXD_Value);	//??????
	SX1276WriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 //?????
	SX1276WriteBuffer( REG_LR_FIFOTXBASEADDR, 0);
	SX1276WriteBuffer( REG_LR_FIFOADDRPTR, 0);
	cmdSwitchEn(enOpen);
	RF_SPI_MasterIO(0x80);
	for (ASM_i = 0; ASM_i < LEN; ASM_i++) {
		RF_SPI_MasterIO(*RF_TRAN_P);
		RF_TRAN_P++;
	}
	cmdSwitchEn(enClose);
	SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0x40);
	SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);
	SX1276LoRaSetOpMode(Transmitter_mode);
}

/*
message content
Lora Gateway --> Device
Mac Header			 Addr					|  Cmd		|		Data		|	check Sum 	|
  1 byte	   |   1 byte				|	1 byte	|		N  byte	|		1byte			|
type|ack|len |
3bit  1   4
*/
#define MAX_LORA_FRAME_LEN 16

typedef __packed  struct _LoarMacFrame {
	uint8_t macheader;
	uint8_t addr;
	uint8_t cmd;
}LoraMacFrame;

static LoraMacFrame *mCurSendingLoraMacFrame = NULL;
#define LORA_GATEWAY_ADDR 0x01

static uint8_t mLoraMyAddr = 0x04;

typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
		RF_TX_WAITACK,
		RF_TX_RESENTING,
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_t;

static RadioState_t mRadioStatus = RF_IDLE;
void print_rssi(char rssi);
void SX1276OnDio0IrqByKarl( void )
{
	int i;
    volatile uint8_t irqFlags = 0;
		//int16_t rssi;
//		int8_t snr = 0;
		uint8_t pktsize = 0;

		irqFlags = SX1276ReadBuffer( REG_LR_IRQFLAGS);	
		//for payload crc
		if (irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) {
			SX1276WriteBuffer( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
		}
		
		//for rxdone
		if (irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) {
			 //read data from fifo
				//rssi = SX1276ReadBuffer( REG_LR_RSSIVALUE );
				//print_string("rx:");
				//print_rssi(rssi);
			
			 pktsize = SX1276ReadBuffer(REG_LR_NBRXBYTES);
       SX1276WriteBuffer( REG_LR_FIFOADDRPTR, SX1276ReadBuffer( REG_LR_FIFORXCURRENTADDR ) );
      // SX1276ReadFifo( RxTxBuffer, pktsize);
				cmdSwitchEn(enOpen);
				RF_SPI_MasterIO(0x00);
				if (pktsize > 100) {
					print_string("pkt size is too big direct throw it");
					return;
				}
				//print_string("got l pack\n");
				for (i = 0; i < pktsize; i++) {
					RxTxBuffer[i] = RF_SPI_READ_BYTE();
				}
				cmdSwitchEn(enClose);
				handle_download_raw_data(RxTxBuffer, pktsize);
				SX1276WriteBuffer( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
				
				SX1276LoRaSetOpMode(Stdby_mode);
				SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); //??????
				SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
				SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
				SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00);
				SX1276LoRaSetOpMode(Receiver_mode);
		}
		
		//for tx done
		if (irqFlags & RFLR_IRQFLAGS_TXDONE_MASK) {
			SX1276WriteBuffer( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
			//radio_tx_done_callback();
			process_post(&process_sx1278, PROCESS_ACTION_SEND_DONE, NULL);
		}
		
		//clear all other irq
		SX1276WriteBuffer( REG_LR_IRQFLAGS, irqFlags );
}

typedef struct
{
    LoraMacFrame **   p_pack_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} LoraPackFifo;

static uint32_t lora_pack_length(LoraPackFifo * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

/**@brief Put one byte to the FIFO. */
static void lora_pack_put(LoraPackFifo * p_fifo, LoraMacFrame *pPack)
{
    p_fifo->p_pack_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = pPack;
    p_fifo->write_pos++;
}

/**@brief Look at one byte in the FIFO. */
static void lora_pack_peek(LoraPackFifo * p_fifo, uint16_t index, LoraMacFrame **pPack)
{
    *pPack = p_fifo->p_pack_buf[(p_fifo->read_pos + index) & p_fifo->buf_size_mask];
}

/**@brief Get one byte from the FIFO. */
static void lora_pack_get(LoraPackFifo * p_fifo, LoraMacFrame **pPack)
{
    lora_pack_peek(p_fifo, 0, pPack);
    p_fifo->read_pos++;
}

#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

static int32_t lora_pack_fifo_init(LoraPackFifo * p_fifo, LoraMacFrame **p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL)
    {
        return -1;
    }

    // Check that the buffer size is a power of two.
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return -1;
    }

    p_fifo->p_pack_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return 0;
}
volatile uint8_t mLoraSendRetry = 0;
/*
* here for uart pack fifo buff
*/
#include "stdlib.h"
#define MAX_UART_FRAME_NUMBER 8
//static LoraMacFrame *gLoraRecvFramePack[MAX_UART_FRAME_NUMBER];
//static LoraPackFifo gLoraRecvPackFifo;

static LoraMacFrame *gLoraSendFramePack[MAX_UART_FRAME_NUMBER];
static LoraPackFifo gLoraSendPackFifo;
void user_call_send_lora_data(uint8_t cmd, uint8_t *data, uint8_t len)
{
	LoraMacFrame *pframe = NULL;
	uint8_t *pbuf;
	uint8_t *pData = NULL;
	uint8_t checksum = 0;
	int i;
	
	if (lora_pack_length(&gLoraSendPackFifo) > (MAX_UART_FRAME_NUMBER/2)) {
		return;
	}
	pframe = (LoraMacFrame*)malloc(len + 4);
	pbuf = (uint8_t *)pframe;
	pData = (uint8_t*)pframe + sizeof(LoraMacFrame);
	
	checksum = 0;
	
	pframe->macheader = 0x10 | len;
	pframe->addr = mLoraMyAddr;
	pframe->cmd  = cmd;
	memcpy(pData, data, len);
	
	for (i = 0; i < (len+3); i++) {
		 checksum += pbuf[i];
	}
	pbuf[i] = checksum;
	
	lora_pack_put(&gLoraSendPackFifo, pframe);
	process_post(&process_sx1278, PROCESS_ACTION_SEND_LORA_DATA, 0);
	
	return;
}

void user_call_send_lora_data_noack(uint8_t cmd, uint8_t *data, uint8_t len)
{
	LoraMacFrame *pframe = NULL;
	uint8_t *pbuf;
	uint8_t *pData = NULL;
	uint8_t checksum = 0;
	int i;
	
	if (lora_pack_length(&gLoraSendPackFifo) > (MAX_UART_FRAME_NUMBER/2)) {
		return;
	}
	pframe = (LoraMacFrame*)malloc(len + 4);
	pbuf = (uint8_t *)pframe;
	pData = (uint8_t*)pframe + sizeof(LoraMacFrame);
	
	checksum = 0;
	
	pframe->macheader = len;
	pframe->addr = mLoraMyAddr;
	pframe->cmd  = cmd;
	memcpy(pData, data, len);
	
	for (i = 0; i < (len+3); i++) {
		 checksum += pbuf[i];
	}
	pbuf[i] = checksum;
	
	lora_pack_put(&gLoraSendPackFifo, pframe);
	process_post(&process_sx1278, PROCESS_ACTION_SEND_LORA_DATA, 0);
	
	return;
}

void send_lora_ack(uint8_t ackcmd)
{
	uint8_t data[1];
	data[0] = ackcmd;
	
	user_call_send_lora_data_noack(0x80, data, 1);
	return;
}
void led_set(uint8_t index, uint8_t val);
static int handle_download_raw_data(uint8_t *buf, int len)
{
	LoraMacFrame *pFrame = (LoraMacFrame *)buf;
	uint8_t *pData = NULL;
	uint8_t framelen = 0;
	uint8_t checksum = 0;
//	uint8_t datalen = 0;
	int i;
	
	if (pFrame->addr != mLoraMyAddr) {
		return 0;
	}
	framelen = pFrame->macheader &0x0f;
	if ((framelen+3) > len) {
		print_string("frame len is not enought\n");
		return 0;
	}
	//len = pFrame->macheader;
	for (i = 0; i < ((pFrame->macheader&0x0f) + 3); i++) {
		checksum+= buf[i];
	}
	
	if (buf[i] != checksum) return -1;
	
	print_string("got 1 pack");
	//got one client lora frame
	if ((mRadioStatus ==  RF_TX_WAITACK) && (mCurSendingLoraMacFrame->addr == pFrame->addr)) {
		mRadioStatus = RF_RX_RUNNING;
		free(mCurSendingLoraMacFrame);
		mCurSendingLoraMacFrame = NULL;
		etimer_set(&mEtimer, CLOCK_SECOND * 10 / 1000);			//10ms will try to send next package
	}
	
	pData = buf +  sizeof(LoraMacFrame);
	switch(pFrame->cmd) {
		case 0x01:
			if (pData[0]) {
				led_set(0, 1);
			} else {
				led_set(0, 0);
			}
			break;
		
		default:
			break;
	}
	//mRadioStatus = RF_RX_RUNNING; //maybe wait ack
	if (pFrame->macheader & LORA_MAC_FRAME_NEED_ACK) {		//need ack
		print_string("lora send ack");
		send_lora_ack(pFrame->cmd);
	}
	return 0;
}


void print_modem_state(uint8_t state);
void print_tx_send_cur_state(uint8_t state);
static void process_get_lorapackage_send(void)
{
	int8_t rssi = 0;
			LoraMacFrame *pFrame = NULL;	
			if(lora_pack_length(&gLoraSendPackFifo) > 0) {
	//			if(SX1276ReadChannelRssi(MODEM_LORA) > -100) {	//signal is high,no need to seed
		//			return;
		//		}
				rssi = SX1276ReadBuffer( REG_LR_MODEMSTAT );
				if (rssi & 0x03) {	//got signal
					etimer_set(&mEtimer, CLOCK_SECOND * 100 /1000);
					print_string("tx1 send busy");
					print_stat_stat(rssi, mRadioStatus);
					if (rssi == 0x32) {
						print_string("lora got 0x32 error in recv mode");
						//tx timeout
						SX1276Reset();
						SX1276LORA_INT();
		
						SX1276LoRaSetOpMode(Stdby_mode);
						SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); //??????
						SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
						SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
						SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00);
						SX1276LoRaSetOpMode(Receiver_mode);
						print_string("reinit lora to recovery lora");
					}
					return;
				}
				//print_string("tx1 send:");
				print_tx_send_cur_state(mRadioStatus);
				lora_pack_get(&gLoraSendPackFifo, &pFrame);
				if (pFrame == NULL) {
					print_string("got send pack null");
					return;
				}

				FUN_RF_SENDPACKET((uint8_t *)pFrame, (pFrame->macheader&0xf)+4);
				//free((void*)pFrame);
				if (mCurSendingLoraMacFrame) free(mCurSendingLoraMacFrame);
				mCurSendingLoraMacFrame = pFrame;
				mRadioStatus = RF_TX_RUNNING;
				mLoraSendRetry = 0;
				etimer_set(&mEtimer, CLOCK_SECOND);
			}
}

static void resend_current_lora_package(void)
{
	LoraMacFrame *pFrame = NULL;
	if (!mCurSendingLoraMacFrame) return;
	
	pFrame = mCurSendingLoraMacFrame;
		
	{
			print_string("tx resend");
				FUN_RF_SENDPACKET((uint8_t *)pFrame, (pFrame->macheader&0xf)+4);
				//free((void*)pFrame);
				//if (mCurSendingLoraMacFrame) free(mCurSendingLoraMacFrame);
				//mCurSendingLoraMacFrame = pFrame;
				mRadioStatus = RF_TX_RUNNING;
			}
}

#define LORA_SEND_MAX_RETRY_TIMES 3


static void handle_lora_timeout(void)
{
	uint8_t rssi;
	
	switch(mRadioStatus) {
		case RF_TX_RUNNING:
			//tx timeout
			SX1276Reset();
			SX1276LORA_INT();
		
			SX1276LoRaSetOpMode(Stdby_mode);
		  SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); //??????
			SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
			SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
			SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00);
			SX1276LoRaSetOpMode(Receiver_mode);
			print_string("tx timeout,and into recv mode");
			mRadioStatus = RF_RX_RUNNING;
			etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 9 +8)%1000+1000)/1000);
			break;
		
		case RF_TX_WAITACK:
			//resend it
			if (mLoraSendRetry < LORA_SEND_MAX_RETRY_TIMES) {
				print_string("tx wait ack timeout resend");
				mLoraSendRetry++;
				//resend_current_lora_package();
				mRadioStatus = RF_TX_RESENTING;
				etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 9 +8)%1000 +1000)/1000);
			} else {
				print_string("tx wait ack timeout throw");
				mLoraSendRetry = 0;
				free(mCurSendingLoraMacFrame);
				mCurSendingLoraMacFrame = NULL;
				mRadioStatus = RF_RX_RUNNING;
				etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 10 +8)%1000+1000)/1000);
			}
			break;
		
		case RF_TX_RESENTING:
		//	print_string("tx resent\n");
				rssi = SX1276ReadBuffer( REG_LR_MODEMSTAT );
				if (rssi & 0x03) {	//got signal
					etimer_set(&mEtimer, CLOCK_SECOND * 100 /1000);
					print_string("tx1 resend busy");
					print_stat_stat(rssi, mRadioStatus);
					if (rssi == 0x32) {
						print_string("lora got 0x32 error in recv mode");
						//tx timeout
						SX1276Reset();
						SX1276LORA_INT();
		
						SX1276LoRaSetOpMode(Stdby_mode);
						SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); //??????
						SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
						SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
						SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00);
						SX1276LoRaSetOpMode(Receiver_mode);
						print_string("reinit lora to recovery lora");
					}
					return;
				}
			resend_current_lora_package();
			//mRadioStatus = RF_TX_WAITACK;
				mRadioStatus = RF_TX_RUNNING;
			etimer_set(&mEtimer, CLOCK_SECOND);
			break;
		
		case RF_IDLE:
		case RF_RX_RUNNING:
		
			process_get_lorapackage_send();
			break;
		
		default:
			break;
	}
}

static void handle_process_send_lora_data(void)
{

	switch(mRadioStatus) {
		case RF_IDLE:
		case RF_RX_RUNNING:
			//print_string("tx send\n");
			process_get_lorapackage_send();
			break;
		
		case RF_TX_RUNNING:
		case RF_TX_WAITACK:
		case RF_TX_RESENTING:
		default:
			break;
	}
}

PROCESS_THREAD(process_sx1278, ev, data)
{
	int ret;
	
  PROCESS_BEGIN();
	
	ret = lora_pack_fifo_init(&gLoraSendPackFifo, gLoraSendFramePack, MAX_UART_FRAME_NUMBER);
	if (ret < 0) {
		return -1;
	}
	SPI_Init1();
	EXTI0_Config();
	
	//register_rf_func(&ctrlTypefunc);
	SX1276Reset();
	
	SX1276LORA_INT();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				SX1276OnDio0IrqByKarl();
				break;
			
			case PROCESS_EVENT_TIMER:
				handle_lora_timeout();
				break;
			
			case PROCESS_ACTION_SEND_LORA_DATA:
				handle_process_send_lora_data();
			/*
				if (mRadioStatus != RF_TX_RUNNING) {
					SX1276SetStby();
					SX1276SetTxConfig(MODEM_LORA,
												20,		//power
												0, 		//fdev
												0,		//bandwith
												7, 		//datarate
												2, 		//coderate
												8, //prelen
												false, //fixlen
												true, //crc
												false, //hop on
												4, 
												false, 	//irinvert
													0x00);
					mSendBuf[0]++;
					SX1276Send((uint8_t *)mSendBuf, 10);
				}
			*/
				break;
			
			case PROCESS_ACTION_SEND_DONE:
					print_string("send done");
					SX1276LoRaSetOpMode(Stdby_mode);
					SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); //??????
					SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
					SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00);
					SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00);
					SX1276LoRaSetOpMode(Receiver_mode);
					if (mCurSendingLoraMacFrame) {
						if (mCurSendingLoraMacFrame->macheader & LORA_MAC_FRAME_NEED_ACK) {
								mRadioStatus = RF_TX_WAITACK;
								etimer_set(&mEtimer, CLOCK_SECOND);
						} else {
							free(mCurSendingLoraMacFrame);
							mCurSendingLoraMacFrame = NULL;
							mRadioStatus = RF_RX_RUNNING;
							etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 10 +8)%1000+500)/1000);
						}
					}
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}

extern struct process process_gpio_key;
void EXTI4_15_IRQHandler(void)
{
//	uint8_t pinVal = 0;
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		process_poll(&process_sx1278);
		EXTI_ClearITPendingBit(EXTI_Line6);
  } else if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
		process_poll(&process_gpio_key);
		EXTI_ClearITPendingBit(EXTI_Line4);
	} else {
		EXTI_ClearITPendingBit(EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15);
	}
}



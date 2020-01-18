/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "contiki.h"
#include "sx1276_spi.h"
#include "user_delay.h"

#include "contiki.h"

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

static uint8_t mLoraMyAddr = 0x01;


/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


PROCESS(process_sx1278, "sx1278 process");

static struct etimer mEtimer;

/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */

/*!
 * \brief Resets the SX1276
 */
void SX1276Reset( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void SX1276SetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode( uint8_t opMode );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( void );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq( void );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq( void );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq( void );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq( void );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq( void );

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1276OnTimeoutIrq( void );

uint16_t SPI_ReadWriteData(uint16_t TxData) ;

void SPI_Init1(void);
/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */
SX1276_t SX1276;

/*!
 * Hardware DIO IRQ callback initialization
 */
/*
 * Radio driver functions implementation
 */
void sx1276_reset_gpio_init(void);
/*
 used pb6 for d0 interruput
 */
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

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6 | EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



static int SX1276Init(void)
{
    uint8_t i;
		uint8_t ReadBuf[2] = {0x00, 0x00};

		sx1276_reset_gpio_init();
		SPI_Init1();
    SX1276Reset();
		
		EXTI0_Config();

		SX1276Write(0x09, 0x0c);
		SX1276Write(0x0a, 0x09);
		SX1276ReadBuffer(0x09, ReadBuf, 2);
		
		if ((ReadBuf[0] == 0x0c) &&
			(ReadBuf[1] == 0x09)) {
		} else {
			 return -1;
		}
  //  RxChainCalibration( );

    SX1276SetOpMode( RF_OPMODE_SLEEP );


    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( MODEM_FSK );

    SX1276.Settings.State = RF_IDLE;
		return 0;
}

RadioState_t SX1276GetStatus( void )
{
    return SX1276.Settings.State;
}

void SX1276SetStatus( RadioState_t status )
{
    SX1276.Settings.State = status;
}
int8_t sx1276_get_last_snr(void) {
	return SX1276.Settings.LoRaPacketHandler.SnrValue;
}
void SX1276SetChannel( uint32_t freq )
{
    SX1276.Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}
 
//modem is in receiver mode
int16_t SX1276ReadChannelRssi( RadioModems_t modem)
{
    int16_t rssi = 0;

		if ((SX1276.Settings.State == RF_TX_WAITACK) || 
			(SX1276.Settings.State == RF_RX_RUNNING)) {

    //Perform carrier sense for maxCarrierSenseTime
    rssi = SX1276ReadRssi( modem );			//检测到有信号强度超标
			} else {
				rssi = -200;
			}
    return rssi;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        DelayMs( 10 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1276Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( );

    return rnd;
}



/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.BandwidthAfc = bandwidthAfc;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.PayloadLen = payloadLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.RxContinuous = rxContinuous;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;
            SX1276.Settings.Fsk.RxSingleTimeout = ( uint32_t )( symbTimeout * ( ( 1.0 / ( double )datarate ) * 8.0 ) * 1000 );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX1276Write( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX1276Write( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if(fixLen)
            {
                SX1276Write( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX1276Write( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

            SX1276Write( REG_PACKETCONFIG1,
                         ( SX1276Read( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.PayloadLen = payloadLen;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1276Write( REG_LR_MODEMCONFIG1,
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX1276Write( REG_LR_MODEMCONFIG3,
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen)
            {
                SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );
            } else {
							  SX1276Write( REG_LR_PAYLOADLENGTH, 255 );
						}

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            } else {
							SX1276Write( REG_LR_HOPPERIOD, 255 );
						}

            if( ( bandwidth == 9 ) && ( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x03 );
            }
            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}


void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | 0x80;
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}


void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX1276SetModem( modem );

    SX1276SetRfTxPower( power );

    switch( modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.Fsk.Power = power;
            SX1276.Settings.Fsk.Fdev = fdev;
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            SX1276Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            SX1276Write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( REG_PREAMBLELSB, preambleLen & 0xFF );

            SX1276Write( REG_PACKETCONFIG1,
                         ( SX1276Read( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            SX1276.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            SX1276Write( REG_LR_MODEMCONFIG1,
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX1276Write( REG_LR_MODEMCONFIG3,
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;
		double rs, ts;
		double tPreamble, tmp;
		double nPayload, tPayload;
		double tOnAir ;

    switch( modem )
    {
    case MODEM_FSK:
        {
           
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( SX1276.Settings.LoRa.Bandwidth )
            {
            case 7: // 125 kHz
                bw = 125000;
                break;
            case 8: // 250 kHz
                bw = 250000;
                break;
            case 9: // 500 kHz
                bw = 500000;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            rs = bw / ( 1 << SX1276.Settings.LoRa.Datarate );
            ts = 1 / rs;
            // time of preamble
            tPreamble = ( SX1276.Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            tmp = ceil( ( 8 * pktLen - 4 * SX1276.Settings.LoRa.Datarate +
                                 28 + 16 * SX1276.Settings.LoRa.CrcOn -
                                 ( SX1276.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( SX1276.Settings.LoRa.Datarate -
                                 ( ( SX1276.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( SX1276.Settings.LoRa.Coderate + 4 );
            nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            tPayload = nPayload * ts;
            // Time on air
            tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor( tOnAir * 1000 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = size;

            if( SX1276.Settings.Fsk.FixLen == false )
            {
                SX1276WriteFifo( ( uint8_t* )&size, 1 );
            }
            else
            {
                SX1276Write( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                SX1276.Settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy( RxTxBuffer, buffer, size );
                SX1276.Settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1276WriteFifo( buffer, SX1276.Settings.FskPacketHandler.ChunkSize );
            SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
            txTimeout = SX1276.Settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX1276.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276Write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( );
                DelayMs( 10 );
            }
            // Write payload buffer
            SX1276WriteFifo( buffer, size );
            txTimeout = SX1276.Settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx( txTimeout );
		SX1276.Settings.State = RF_TX_RUNNING;
}

void SX1276SetSleep( void )
{
    SX1276SetOpMode( RF_OPMODE_SLEEP );
  //  SX1276.Settings.State = RF_IDLE;
}

void SX1276SetStby( void )
{
    SX1276SetOpMode( RF_OPMODE_STANDBY );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276SetRx( uint32_t timeout )
{
    bool rxContinuous = false;

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = SX1276.Settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

            SX1276Write( REG_DIOMAPPING2, ( SX1276Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read( REG_FIFOTHRESH ) & 0x3F;

            SX1276Write( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX1276.Settings.LoRa.Bandwidth < 9 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1276Write( REG_LR_TEST30, 0x00 );
                switch( SX1276.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x48 );
                    SX1276SetChannel(SX1276.Settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX1276.Settings.LoRa.RxContinuous;

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    if( SX1276.Settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
          //  TimerSetValue( &RxTimeoutSyncWord, SX1276.Settings.Fsk.RxSingleTimeout );
        //    TimerStart( &RxTimeoutSyncWord );
        }
    }
    else
    {
       // if( rxContinuous == true )
			 if (true) 
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            //SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
		SX1276.Settings.State = RF_RX_RUNNING;
}

void SX1276SetTx( uint32_t timeout )
{
   // TimerSetValue( &TxTimeoutTimer, timeout );

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );

            SX1276Write( REG_DIOMAPPING2, ( SX1276Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1276StartCad( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX1276SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX1276SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )( time * 1000 );

    SX1276SetChannel( freq );

    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    
    SX1276Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX1276Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX1276Read( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) 
        {
            rssi = RSSI_OFFSET_HF + SX1276Read( REG_LR_RSSIVALUE );
        }
        else
        {
            rssi = RSSI_OFFSET_LF + SX1276Read( REG_LR_RSSIVALUE );
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void sx1276_reset_gpio_init(void)
{
}

void SX1276Reset( void )
{
	 GPIOB->BSRR = GPIO_Pin_7;
	 GPIOB->BRR = GPIO_Pin_7;
		DelayMs(10);
	GPIOB->BSRR = GPIO_Pin_7;
		DelayMs(100);
}

void SX1276SetOpMode( uint8_t opMode )
{
    if( opMode == RF_OPMODE_SLEEP )
    {
       // SX1276SetAntSwLowPower( true );
    }
    else
    {
      //  SX1276SetAntSwLowPower( false );
      //  SX1276SetAntSw( opMode );
    }
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276SetModem( RadioModems_t modem )
{
    if( ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        SX1276.Settings.Modem = MODEM_LORA;
    }
    else
    {
        SX1276.Settings.Modem = MODEM_FSK;
    }

    if( SX1276.Settings.Modem == modem )
    {
        return;
    }

    SX1276.Settings.Modem = modem;
    switch( SX1276.Settings.Modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetSleep( );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetSleep( );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}


unsigned char SpiRCVaByte(void);
void SpiInOut(unsigned char senddata);
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t i;

	//NSS = 0;
	//    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
	SET_L();
	SpiInOut( addr | 0x80 );
	for( i = 0; i < size; i++ )
	{
		SpiInOut( buffer[i] );
	}
	SET_H();
	//NSS = 1;
	//GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t i;

	//NSS = 0;
	//GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
SET_L();
	SpiInOut( addr & 0x7F );

	for( i = 0; i < size; i++ )
	{
		buffer[i] = SpiRCVaByte();
	}
SET_H();
	//NSS = 1;
	//GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );

}


#if 1
void SX1276Write( uint8_t addr, uint8_t data )
{
	SX1276WriteBuffer( addr, &data, 1 );
}

uint8_t SX1276Read( uint8_t addr)
{
	uint8_t data;
	SX1276ReadBuffer( addr, &data, 1 );
	return data;
}

#endif

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
	SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
	SX1276ReadBuffer( 0, buffer, size );
}


void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( SX1276.Settings.Fsk.FixLen == false )
        {
            SX1276Write( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1276Write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1276SetPublicNetwork( bool enable )
{

    SX1276SetModem( MODEM_LORA );
    SX1276.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

volatile uint8_t mRxBufTest[12];

void led_set(uint8_t index, uint8_t val);
int handle_download_raw_data(uint8_t *buf, int len);

void radio_tx_done_callback(void)
{
}
void notify_last_snr_value(int8_t snr)
{
}
void SX1276OnDio0IrqByKarl( void )
{
    volatile uint8_t irqFlags = 0;
		int16_t rssi;
		int8_t snr = 0;
		uint8_t pktsize = 0;
    
		irqFlags = SX1276Read(REG_LR_IRQFLAGS);
		
		//for payload crc
		if (irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) {
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
		}
		
		//for rxdone
		if (irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) {
			//get signal strength
			snr = SX1276Read( REG_LR_PKTSNRVALUE );
			notify_last_snr_value(SX1276.Settings.LoRaPacketHandler.SnrValue);
      if( snr ) {
				// Invert and divide by 4
        snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
        snr = -snr;
      } else {
				// Divide by 4
        snr = ( snr & 0xFF ) >> 2;
      }
			
			 //read data from fifo
			 pktsize = SX1276Read( REG_LR_RXNBBYTES );
       SX1276Write( REG_LR_FIFOADDRPTR, SX1276Read( REG_LR_FIFORXCURRENTADDR ) );
       SX1276ReadFifo( RxTxBuffer, pktsize);
				handle_download_raw_data(RxTxBuffer, pktsize);
				SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
		}
		
		//for tx done
		if (irqFlags & RFLR_IRQFLAGS_TXDONE_MASK) {
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
			//radio_tx_done_callback();
			process_post(&process_sx1278, PROCESS_ACTION_SEND_DONE, NULL);
		}
		
		//clear all other irq
		SX1276Write( REG_LR_IRQFLAGS, irqFlags );
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
static LoraMacFrame *gLoraRecvFramePack[MAX_UART_FRAME_NUMBER];
static LoraPackFifo gLoraRecvPackFifo;

static LoraMacFrame *gLoraSendFramePack[MAX_UART_FRAME_NUMBER];
static LoraPackFifo gLoraSendPackFifo;
void user_call_send_lora_data(uint8_t saddr, uint8_t cmd, uint8_t *data, uint8_t len)
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
	pframe->addr = saddr;
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

void user_call_send_lora_data_noack(uint8_t saddr, uint8_t cmd, uint8_t *data, uint8_t len)
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
	pframe->addr = saddr;
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

void send_lora_ack(uint8_t saddr, uint8_t ackcmd)
{
	uint8_t data[1];
	data[0] = ackcmd;
	
	user_call_send_lora_data_noack(saddr, 0x80, data, 1);
	return;
}

int handle_download_raw_data(uint8_t *buf, int len)
{
	LoraMacFrame *pFrame = (LoraMacFrame *)buf;
	uint8_t *pData = NULL;
	uint8_t checksum = 0;
	uint8_t datalen = 0;
	int i;
	
//gateway all data will received
//	if (pFrame->addr != mLoraMyAddr) {
//		return 0;
//	}
	
	len = pFrame->macheader;
	
	for (i = 0; i < ((pFrame->macheader&0x0f) + 3); i++) {
		checksum+= buf[i];
	}
	
	if (buf[i] != checksum) return -1;
	
	//got one client lora frame
	if ((SX1276.Settings.State ==  RF_TX_WAITACK) && (mCurSendingLoraMacFrame->addr == pFrame->addr)) {
		SX1276.Settings.State = RF_RX_RUNNING;
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
	//SX1276.Settings.State = RF_RX_RUNNING; //maybe wait ack
	if (pFrame->macheader & LORA_MAC_FRAME_NEED_ACK) {		//need ack
		send_lora_ack(pFrame->addr, pFrame->cmd);
	}
	return 0;
}

static void process_get_lorapackage_send(void)
{
			LoraMacFrame *pFrame = NULL;	
			if(lora_pack_length(&gLoraSendPackFifo) > 0) {
				if(SX1276ReadChannelRssi(MODEM_LORA) > -100) {	//signal is high,no need to seed
					return;
				}
				
				lora_pack_get(&gLoraSendPackFifo, &pFrame);
				if (pFrame == NULL) return;
				
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
				SX1276Send((uint8_t *)pFrame, (pFrame->macheader&0xf)+4);
				//free((void*)pFrame);
				if (mCurSendingLoraMacFrame) free(mCurSendingLoraMacFrame);
				mCurSendingLoraMacFrame = pFrame;
				SX1276.Settings.State = RF_TX_RUNNING;
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
				SX1276Send((uint8_t *)pFrame, (pFrame->macheader&0xf)+4);
				//free((void*)pFrame);
				//if (mCurSendingLoraMacFrame) free(mCurSendingLoraMacFrame);
				//mCurSendingLoraMacFrame = pFrame;
				SX1276.Settings.State = RF_TX_RUNNING;
			}
}

#define LORA_SEND_MAX_RETRY_TIMES 3


static void handle_lora_timeout(void)
{
	switch(SX1276.Settings.State) {
		case RF_TX_RUNNING:
			//tx timeout
			SX1276SetStby();
				//SX1276SetPublicNetwork(true);
				//SX1276SetChannel(471700000);
				//SX1276SetRxConfig(MODEM_LORA, 0,  7, 2, 0, 8, 0x3ff, false, 8, true, false, 4, false, true);
			SX1276SetRxConfig(MODEM_LORA, 0, 					7, 				2, 				0, 						8,						0x3ff, 			false, 		8, 					true, 	false, 		4, 					false, 	true);

			SX1276SetRx(0x00);
			SX1276.Settings.State = RF_RX_RUNNING;
			etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 9 +8)%1000)/1000);
			break;
		
		case RF_TX_WAITACK:
			//resend it
			if (mLoraSendRetry < LORA_SEND_MAX_RETRY_TIMES) {
				mLoraSendRetry++;
				//resend_current_lora_package();
				SX1276.Settings.State = RF_TX_RESENTING;
				etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 9 +8)%1000)/1000);
			} else {
				mLoraSendRetry = 0;
				free(mCurSendingLoraMacFrame);
				mCurSendingLoraMacFrame = NULL;
				SX1276.Settings.State = RF_RX_RUNNING;
				etimer_set(&mEtimer, CLOCK_SECOND * ((mLoraMyAddr * 10 +8)%1000)/1000);
			}
			break;
		
		case RF_TX_RESENTING:
			resend_current_lora_package();
			SX1276.Settings.State = RF_TX_WAITACK;
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

	switch(SX1276.Settings.State) {
		case RF_IDLE:
		case RF_RX_RUNNING:
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
	
	SX1276Init();
	SX1276SetStby();
	SX1276SetChannel(471700000);
	SX1276SetPublicNetwork(true);
	
										//modem     bandwidth   datarate  coderate	bandwidhtafc	preambleLen		symbtimeut	fixlen		payloadlen	crcon		freqhop		hopperiod		iqinvert	rxcontinue
	SX1276SetRxConfig(MODEM_LORA, 0, 					7, 				2, 				0, 						8,						0x3ff, 			false, 		8, 					true, 	false, 		4, 					false, 	true);
	SX1276SetRx(0x00);
	
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
				if (SX1276.Settings.State != RF_TX_RUNNING) {
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
				SX1276SetStby();
				//SX1276SetPublicNetwork(true);
				//SX1276SetChannel(471700000);
				//SX1276SetRxConfig(MODEM_LORA, 0,  7, 2, 0, 8, 0x3ff, false, 8, true, false, 4, false, true);
				SX1276SetRxConfig(MODEM_LORA, 0, 					7, 				2, 				0, 						8,						0x3ff, 			false, 		8, 					true, 	false, 		4, 					false, 	true);

				SX1276SetRx(0x00);
				if (mCurSendingLoraMacFrame) {
					if (mCurSendingLoraMacFrame->macheader & LORA_MAC_FRAME_NEED_ACK) {
							SX1276.Settings.State = RF_TX_WAITACK;
					} else {
						free(mCurSendingLoraMacFrame);
						mCurSendingLoraMacFrame = NULL;
						SX1276.Settings.State = RF_RX_RUNNING;
					}
				}

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



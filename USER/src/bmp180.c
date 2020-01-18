#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <bmp180.h>

#include "saferopemachine.h"

/*
GPB5   scl
GPB6   sda
*/
#define SET_SDA_INPUT()		do {GPIOB->MODER &= ~(0x03 << 12); }while(0)
#define SET_SDA_OUTPUT()	do {temp = GPIOB->MODER; temp &= ~(0x03 << 12); GPIOB->MODER = temp | (0x1 << 12);} while(0)
#define SET_SDA_OUT1()		GPIOB->BSRR |= (1<<6)
#define SET_SDA_OUT0()		GPIOB->BRR |= (1<<6)

#define GET_SDA_VAL()		 (GPIOB->IDR & (1<<6))?1:0

#define SET_SCL_OUTPUT()	do {GPIOB->MODER &= ~(0x03 << 10); GPIOB->MODER |= (0x1 << 10);} while(0)
#define SET_SCL_OUT1()		GPIOB->BSRR |= (1<<5)
#define SET_SCL_OUT0()		GPIOB->BRR |= (1<<5)


// addresses of the device
#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.


#define I2C_ADR_W (MS561101BA_ADDR_CSB_LOW << 1)  //0xee
#define I2C_ADR_R ((MS561101BA_ADDR_CSB_LOW << 1) | 0x01)


static void delay()
{
	volatile int j;
	for(j = 0; j < 100; j++);
}

void i2c_init(void)
{
	GPIO_InitTypeDef GpioInitStructure;
	uint32_t temp;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	//gpio8 gpio9 config
	GpioInitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GpioInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GpioInitStructure.GPIO_OType = GPIO_OType_PP;
	GpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GpioInitStructure);
	
	SET_SDA_OUTPUT();
	SET_SCL_OUTPUT();
	SET_SDA_OUT1();
	SET_SCL_OUT1();
}
static void i2c_start()
{  
	uint32_t temp;
	delay(); 
	delay(); 
	//初始化GPIO口  
	SET_SDA_OUTPUT();              //设置SDA方向为输出    
	SET_SDA_OUT1();                //设置SDA为高电平  
	SET_SCL_OUT1();                 //设置SCL为高电平  
	delay();                            //延时  
	//起始条件  
	SET_SDA_OUT0();                 //SCL为高电平时，SDA由高变低  
	delay();  
}  
/* I2C终止条件 */  
static void i2c_stop()  
{  
	uint32_t temp;

	SET_SDA_OUTPUT(); 
	SET_SDA_OUT0(); 
	delay();  	
	SET_SCL_OUT1();  
	delay();  
	SET_SDA_OUT1();             //SCL高电平时，SDA由低变高  
	delay();  
}  

/* I2C发出ACK信号(读数据时使用) */  
static void i2c_send_ack()  
{  
	uint32_t temp;
	
	SET_SDA_OUTPUT();          //设置SDA方向为输出  
	SET_SDA_OUT0();             //发出ACK信号  
	delay(); 
	SET_SCL_OUT1();              // SCL变高
	delay();  
	SET_SCL_OUT0();              // SCL变低
	delay();  
}  

static void i2c_send_nack()  
{  
	uint32_t temp;
	SET_SDA_OUTPUT();          //设置SDA方向为输出  
	SET_SDA_OUT1();             //发出ACK信号  
	delay(); 
	SET_SCL_OUT1();              // SCL变高  
	delay();  
	SET_SCL_OUT0();              // SCL变低  
	delay();  
}  

/* I2C字节写 */  
static int i2c_write_byte(unsigned char b)  
{  
	int i, ack;  
	uint32_t temp;
	
	SET_SDA_OUTPUT();          //设置SDA方向为输出  
	SET_SCL_OUT0();
	delay();
	for (i=0; i<8; i++) {  
		if (b & 0x80) {
			//从高位到低位依次准备数据进行发送  
			SET_SDA_OUT1();
		} else {
			SET_SDA_OUT0();
		}
		delay();
		SET_SCL_OUT1();             // SCL变低  
		delay();  
		b <<= 0x01;
		SET_SCL_OUT0();             // SCL变高  
		delay();  
	}  
	//检查目标设备的ACK信号  
	SET_SDA_INPUT();           //设置SDA方向为输入  
	delay();  
	//delay();  
	SET_SCL_OUT1();              // SCL变高  
	delay();
	ack = GET_SDA_VAL();                //读取ACK信号  

	delay();  
	SET_SCL_OUT0();
	SET_SDA_OUTPUT();
	SET_SDA_OUT0();

	delay();
	return ack;
}  

/* I2C字节读 */  
static unsigned char i2c_read_byte()  
{  
	int i;  
	unsigned char r = 0;  
	SET_SDA_INPUT();           //设置SDA方向为输入  
	for (i = 0; i < 8; i++) {
		SET_SCL_OUT1();         // SCL变低  
		delay(); 
		r <<=1;
		r |=GET_SDA_VAL();      //从高位到低位依次准备数据进行读取  
		SET_SCL_OUT0();         // SCL变高  
		delay();  
	}  
	i2c_send_ack();                 //向目标设备发送ACK信号  
	return r;  
}  

static unsigned char i2c_read_byte_nack()  
{  
	int i;  
	unsigned char r = 0;  
	SET_SDA_INPUT();           //设置SDA方向为输入  

	for (i=0;i<8; i++) {  
		SET_SCL_OUT1();         // SCL变  
		delay();  
		r <<=1;
		r |= GET_SDA_VAL();      //从高位到低位依次准备数据进行读取  
		SET_SCL_OUT0();         // SCL变
		delay();  
	}  
	i2c_send_nack();                 //向目标设备发送ACK信号  
	return r;  
}  


static int ms5611_send_command(unsigned char cmd)
{
	int ack;

	i2c_start();

	ack = i2c_write_byte(I2C_ADR_W);
	if (ack == 1) {
		return  -1;
	}

	ack = i2c_write_byte(cmd);
	if (ack == 1) {
		i2c_stop();
		return  -1;
	} else {
		i2c_stop();
		return 0;
	}
}

static int ms5611_read_uint16_data(uint16_t *pu16)
{
	int ack;
//	unsigned char val,val2;
	
	i2c_start();
	
	ack = i2c_write_byte(I2C_ADR_R);
	if (ack == 1) {
		return -1;
	}
	*pu16 = i2c_read_byte();
	*pu16 = (*pu16) << 8;
	
	*pu16 += i2c_read_byte_nack();
	
	i2c_stop();
	return 0;
}


static int ms5611_read_uint24_data(uint32_t *pu32)
{
	int ack;
//	unsigned char val,val2;
	
	i2c_start();
	
	ack = i2c_write_byte(I2C_ADR_R);
	if (ack == 1) {
		return -1;
	}
	
	*pu32 = i2c_read_byte();
	*pu32 = (*pu32) << 8;
	*pu32 += i2c_read_byte();
	*pu32 = (*pu32) << 8;
	*pu32 += i2c_read_byte_nack();
	
	i2c_stop();
	return 0;
}

// Read the uncompensated temperature value
extern void DelayMs(uint32_t nTime);
void print_rawdata(uint32_t data1);
static int ms5611_do_conversion(uint32_t *pdata, uint8_t cmd)
{
	int ret;
	
	ret = ms5611_send_command(cmd);
	if (ret < 0) {
		return -1;
	}
	
	//delay for cover
	DelayMs(15);
	
	//read data command
	ret = ms5611_send_command(0x00);
	if (ret < 0) {
		return -1;
	}
	
	ret = ms5611_read_uint24_data(pdata);
	if(ret < 0) {
		return -1;
	}
	print_rawdata(*pdata);
	return 0;
}

static int read_raw_temp(uint8_t osr, uint32_t *temp)
{
	return ms5611_do_conversion(temp, MS561101BA_D2 + osr);
}

static int read_raw_pressure(uint8_t osr, uint32_t *pre)
{
	return ms5611_do_conversion(pre, MS561101BA_D1 + osr);
}



static uint16_t g_ms5611_id = 0x0000;
static uint16_t g_coeff[6];
static int getDeltaTemp(uint8_t OSR, int32_t *pint64) {
	uint32_t rawtemp;
	int ret;
	
	ret = read_raw_temp(OSR, &rawtemp);
	if (ret < 0) {
		return -1;
	} 
  *pint64 = (int32_t)rawtemp - (((int32_t)g_coeff[4]) << 8);  //d2-c5 * power(2, 8)
	return 0;
}

void print_temp(float data);
#define EXTRA_PRECISION 5
static int getPressure(uint8_t OSR, float *fpre, float *ptemp) {
  // see datasheet page 7 for formulas
	uint32_t rawpre;
//	int64_t delttemp; 
	int ret;
  int32_t dT;
	int64_t off, sens;
	int64_t T2, Aux_64, OFF2, SENS2;
	int64_t TEMP;
	
	ret = getDeltaTemp(OSR, &dT);
	if (ret < 0) {
		return -1;
	}
	
	*ptemp =  ((1<<EXTRA_PRECISION)*(uint32_t)2000 + ((dT * g_coeff[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION) * 100.0);
	//return 0;
	print_temp(*ptemp);
  off  = (((int64_t)g_coeff[1]) << 16) + ((((int64_t)g_coeff[3]) * dT) >> 7); //C2 * power(2,16) + c4 * dt/ power(2,7)
  sens = (((int64_t)g_coeff[0]) << 15) + ((((int64_t)g_coeff[2]) * dT) >> 8);//c1 * power(2,15) + c3 * dt/ power(2,8)
/*
	if (*ptemp < 20.0) { // here pensense
		T2 = (((int64_t)dT) * dT) >> 31;
		TEMP = *ptemp * 100;
		Aux_64 = ((int64_t)(TEMP - 2000)) * (TEMP - 2000);
		OFF2 = (5 * Aux_64) >> 1;
		SENS2 = (5 * Aux_64) >> 2;
		
		*ptemp = *ptemp - (float)T2/100.0;
		off = off - OFF2;
		sens = sens - SENS2;
		
		print_temp(*ptemp);
	}	
	
	*/
	ret = read_raw_pressure(OSR, &rawpre);
	if (ret < 0) {
		return ret;
	}
  *fpre = (((((int64_t)rawpre * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION) * 100.0);
	return 0;
}

int getTemperature(uint8_t OSR, float *ptemp) {
  // see datasheet page 7 for formulas
	int ret;
  int32_t dT;
	ret = getDeltaTemp(OSR, &dT);
	if (ret < 0) {
		return -1;
	}
  *ptemp =  ((1<<EXTRA_PRECISION)*(uint32_t)2000 + ((dT * g_coeff[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION) * 100.0);
	return 0;
}

void print_string(char *str);

static float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
 // C = C /0.0000225577;
	 C = C  /0.0000225577;
	//C *= 10.0;

  return C;
}
//volatile 	float altitude;
void print_presure(float data);
static int get_altitude(float *altitude)
{
	int ret;
	float temp;
	float presure;
	
	const float sea_press = 1013.25;
	
	/*
	ret = getTemperature(MS561101BA_OSR_4096, &temp);
	if (ret < 0) {
		return ret;
	}
	*/
	ret = getPressure(MS561101BA_OSR_4096, &presure, &temp);
	if (ret < 0) {
		return ret;
	}
	#if 1
	presure *= 100;
//print_presure(presure);
//	presure = presure / 101325.0; // "standard atmosphere"
	
	*altitude = calcAltitude(presure);
	
	#else 
	*altitude = ((pow((sea_press / presure), 1.0/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	#endif
//	print_temp(temp);
	return 0;
}

void print_coff(int index, uint16_t data);
static int ms5611_init()
{
	int ret;
	int i;
	//send reset command
	
	ret = ms5611_send_command(0x1e);
	if (ret < 0) {
		return -1;
	}
	
	DelayMs(10);
	
	//get manufactore id
	ret = ms5611_send_command(0xa0);
	if (ret < 0) {
		return -1; 	
	}
	
	ret = ms5611_read_uint16_data(&g_ms5611_id);
	if (ret < 0) {
		return -1;
	}
	
	//read coff
	for (i = 0; i < 6; i++) {
		ret = ms5611_send_command(0xa0 | ((i+1)<<1));
		if (ret < 0) {
			return -1;
		}
		ret = ms5611_read_uint16_data(&g_coeff[i]);
		if (ret < 0) {
			return -1;
		}
	}
	
  for (i = 0; i < 6; i++) {
		print_coff(i, g_coeff[i]);
	}
	return 0;
}



static unsigned char gId = 0x00;
volatile float altitude;
volatile float temperature;

PROCESS(process_bmp180, "bmp180 process");
static struct etimer gBmpReadTimer;

#define MAX_PER_TIME_SAMPLES 4
#define MAX_ALTITUDE_BUF_SIZE 4
volatile  float galtitudeHistory[MAX_ALTITUDE_BUF_SIZE]; //will record 10 
volatile  float galtitudeNow[MAX_PER_TIME_SAMPLES]; //will record 4 temparay meter
volatile uint8_t galtitudeNowCurIndex = 0; 
volatile uint8_t galtitudeHistoryCurIndex = 0;


volatile uint32_t gAltitudeActionFlags = 0x00;
volatile uint32_t gAltitudeCurStatus = 0x00;
volatile float gAltitudeH0 = 0.0;

extern struct process process_safemachine;

bool IsInLowMode(void)
{
	if (gAltitudeCurStatus & ALTITUDE_STATUS_HIGH) {
		return false;
	} else {
		return true;
	}
}
void print_float(float data);
void print_string(char *str);
void print_floatH0(float data);
static void bmp180_push_one_sample(float altitude)
{
	int i;
	float mcount = 0.0;
	int mcount1 = 0;
	if (galtitudeHistoryCurIndex < MAX_ALTITUDE_BUF_SIZE) {
		galtitudeHistory[galtitudeHistoryCurIndex++] = altitude;
		return;
	} else {
		memcpy((void*)&galtitudeHistory, (void*)(&galtitudeHistory[1]), sizeof(float) *  (MAX_ALTITUDE_BUF_SIZE-1));
		galtitudeHistory[MAX_ALTITUDE_BUF_SIZE-1] = altitude;
	}
	//here calculate the altitude
	print_float(altitude);
	print_floatH0(gAltitudeH0);
	if (ALTITUDE_ACTION_H0_SET & gAltitudeActionFlags) { //first set H0, 
		gAltitudeActionFlags &=~ALTITUDE_ACTION_H0_SET;
		gAltitudeH0  = altitude;
		process_post(&process_safemachine, SAFE_MACHINE_EVENT_TOLOW, NULL);
		print_string("h0 reset");
		for (i = 0; i < MAX_ALTITUDE_BUF_SIZE; i++) {
			galtitudeHistory[i] = altitude;
		}
		print_float(altitude);
	} else {
		//H0 is set, then check current altitude
		if (gAltitudeCurStatus & ALTITUDE_STATUS_HIGH) {//当前高，需要检测是否低,如果超过5m，则要设置H5标志
			mcount = 0;
			
			if (!(gAltitudeCurStatus & ALTITUDE_STATUS_HIGH5)) {//高空中如果高度 >5.1m，则设置超过5m标志
				for (i = 0; i < (galtitudeHistoryCurIndex); i++) {
					mcount += (galtitudeHistory[i] - gAltitudeH0);
				}
				mcount = fabs(mcount);
				if ((mcount/galtitudeHistoryCurIndex) > 5.2) {
					gAltitudeCurStatus |= ALTITUDE_STATUS_HIGH5;
					led_blink(2, 5, 200, 200);//karl
				}
				print_float(mcount/galtitudeHistoryCurIndex);
				if ((mcount/galtitudeHistoryCurIndex) < 1.8) {
					gAltitudeCurStatus &= ~(ALTITUDE_STATUS_HIGH | ALTITUDE_STATUS_HIGH5);
					//here report low
					//	led_set('0', '1');
					//led_set('1', '0');
					process_post(&process_safemachine, SAFE_MACHINE_EVENT_TOLOW, NULL);
				}
			} else {//达到过5.1米，则小于4.9米就正常
				mcount = 0;
				for (i = 0; i < (galtitudeHistoryCurIndex); i++) {
					mcount += (galtitudeHistory[i] - gAltitudeH0);
				}
				mcount = fabs(mcount);
				print_float(mcount/galtitudeHistoryCurIndex);
				if ((mcount/galtitudeHistoryCurIndex) < 4.8) {
					gAltitudeCurStatus &= ~ALTITUDE_STATUS_HIGH;
					//here report low
					//	led_set('0', '1');
					//led_set('1', '0');
					process_post(&process_safemachine, SAFE_MACHINE_EVENT_TOLOW, NULL);
				}
			}
		} else {
			if (gAltitudeCurStatus & ALTITUDE_STATUS_HIGH5) {				//由超过5m进入的低空,需要超过5.1才能进入高空
				mcount = 0;
				for (i = 0; i < (galtitudeHistoryCurIndex); i++) {
					mcount += (galtitudeHistory[i] - gAltitudeH0);
				}
				mcount = fabs(mcount);
				print_float(mcount/galtitudeHistoryCurIndex);
				if ((mcount/galtitudeHistoryCurIndex) > 5.2) {
					gAltitudeCurStatus |= ALTITUDE_STATUS_HIGH;
					//here report high
					//led_set('1', '1');
					//led_set('0', '0');
					process_post(&process_safemachine, SAFE_MACHINE_EVENT_TOHIGH, NULL);
				}

				if ((mcount/galtitudeHistoryCurIndex) < 1.8) {
					gAltitudeCurStatus &= ~ALTITUDE_STATUS_HIGH5;
				}
			} else {		//未进入过5m高空
				mcount = 0;
				for (i = 0; i < (galtitudeHistoryCurIndex); i++) {
					mcount += (galtitudeHistory[i] - gAltitudeH0);
				}
				mcount = fabs(mcount);
				print_int_int(i, galtitudeHistoryCurIndex);
				//print_float(gAltitudeH0);
				print_float(mcount/galtitudeHistoryCurIndex);
				if ((mcount/galtitudeHistoryCurIndex) > 2.2) {
					gAltitudeCurStatus |= ALTITUDE_STATUS_HIGH;
					//here report high
					//led_set('1', '1');
					//led_set('0', '0');
					process_post(&process_safemachine, SAFE_MACHINE_EVENT_TOHIGH, NULL);
				}
			}
		}
	}
	if (ALTITUDE_ACTION_SPEED_DETECT & gAltitudeActionFlags) {
		if (galtitudeHistoryCurIndex < 2) return;
		mcount1 = 0;
		for (i = 0; i < (galtitudeHistoryCurIndex-1); i++) {
			if (fabs(galtitudeHistory[i+1] - galtitudeHistory[i]) > 3.0) {
				mcount1++;
				break;
			}
		}
		
		if (mcount1 && (!(gAltitudeCurStatus & ALTITUDE_STATUS_DROP))) {
			gAltitudeCurStatus |=  ALTITUDE_STATUS_DROP;
			process_post(&process_safemachine, SAFE_MACHINE_EVENT_DROP, NULL);
			//report drop
		} else if (!mcount1 && (gAltitudeCurStatus & ALTITUDE_STATUS_DROP)) {
			gAltitudeCurStatus &=  ~ALTITUDE_STATUS_DROP;
			//report drop 恢复
		}
	}
}
void print_int2_float(int data1, float data2);

static float bmp180_atitude_fliter(void)
{
	static float lastvalue = 0.0;
	static uint8_t errortimes = 0;
	int validcount = 0;
	float sumvalue = 0.0;
	int i;
	
	if (fabs(lastvalue) < 1.0) {//first time
		for (i = 0; i < MAX_PER_TIME_SAMPLES; i++) {
			sumvalue += galtitudeNow[i];
		}
		sumvalue = sumvalue / 4;
		lastvalue = sumvalue;
		return sumvalue;
	} else {
		for (i = 0; i < MAX_PER_TIME_SAMPLES; i++) {
			if (fabs(lastvalue - galtitudeNow[i]) < 5.0) {
				sumvalue += galtitudeNow[i];
				validcount++;
			}
		}
		if (validcount >= 1) {
			sumvalue = sumvalue / validcount;
			lastvalue = sumvalue;
			errortimes = 0x00;
		} else {
			errortimes++;
			if (errortimes > 3) {
				//this will take new value
				print_string("bmp180 init alititude error, take this new one!!!!\r\n");
				for (i = 0; i < MAX_PER_TIME_SAMPLES; i++) {
					if (1) {
						sumvalue += galtitudeNow[i];
						validcount++;
					}
				}
				lastvalue = sumvalue/validcount;
				gAltitudeActionFlags |= ALTITUDE_ACTION_H0_SET;
			}
			return lastvalue;
		}
	}
	return lastvalue;
}
void print_fliter(float data);
static void bmp180_sample_process(void)
{
	//int i;
	int ret;
	float allaltitude;
	float altitude;
	
	if (gAltitudeActionFlags) {
		ret = get_altitude(&altitude);
		if (ret < 0) {
			altitude = 50;
		}
		galtitudeNow[galtitudeNowCurIndex] = altitude; //calcAltitude(pressure); //get one altitude
		//print_string("raw get\r\n");
		print_int2_float(galtitudeNowCurIndex, galtitudeNow[galtitudeNowCurIndex]);
		
		if (galtitudeNowCurIndex >= (MAX_PER_TIME_SAMPLES-1)) { //4 times samples ok
			/*
			for (i = 0; i < MAX_PER_TIME_SAMPLES; i++) {
				allaltitude += galtitudeNow[i];
			}
			*/
			allaltitude = bmp180_atitude_fliter();
			print_fliter(allaltitude);
			bmp180_push_one_sample(allaltitude);
			etimer_set(&gBmpReadTimer, CLOCK_SECOND );
			galtitudeNowCurIndex = 0x00;
		} else {
			galtitudeNowCurIndex++;
			etimer_set(&gBmpReadTimer, CLOCK_SECOND/10);
		}
	} else {
		print_string("bmp stop by itself\r\n");
	}
}

void start_measure_altitude(uint32_t action)
{
//	if(gId != 0x55) return;
	
	if (gAltitudeActionFlags) {
		print_string("bmp180 timer already start\r\n");
		gAltitudeActionFlags |= action;
		return;
	}
	
	memset((void*)galtitudeNow, 0x00, sizeof(galtitudeNow));
	memset((void*)galtitudeHistory, 0x00, sizeof(galtitudeHistory));
	
	gAltitudeActionFlags |= action;
	
	galtitudeNowCurIndex = 0;
	galtitudeHistoryCurIndex = 0;
	PROCESS_CONTEXT_BEGIN(&process_bmp180);
	etimer_set(&gBmpReadTimer, CLOCK_SECOND/10);
	PROCESS_CONTEXT_END(&process_bmp180);
	print_string("bmp timer start\r\n");
}

void stop_measure_altitude(void)
{
	gAltitudeActionFlags = 0x00;
	PROCESS_CONTEXT_BEGIN(&process_bmp180);
	etimer_stop(&gBmpReadTimer);	
	PROCESS_CONTEXT_END(&process_bmp180);
}

PROCESS_THREAD(process_bmp180, ev, data)
{
	int i;
  PROCESS_BEGIN();
	i2c_init();

	ms5611_init();
	
	//for (i = 0; i < 100; i++) 
	//test_ms5611();
	
	start_measure_altitude(ALTITUDE_ACTION_H0_SET);
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				//print_string("bmp sample once\r\n");
				bmp180_sample_process();
				break;
			
			case BMP180_PROCESS_EVENT_RESETH0:
				start_measure_altitude(ALTITUDE_ACTION_H0_SET);
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}



#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "user_usart1.h"

PROCESS(process_hw_timer, "led process");

enum {
	PROCESS_ACTION_I_SAMPLED  = 1,
	PROCESS_ACTION_P_SAMPLED,
	PROCESS_ACTION_CAL_PREPARE,
	PROCESS_ACTION_CAL_START,
	PROCESS_ACTION_UPDATE_POWER,
	PROCESS_ACTION_CAL_END,
	PROCESS_ACTION_SQUREWAVE_REPORT,
	PROCESS_ACTION_NOSQUREWAVE_REPORT,
};
#define D_TIME1_100MS  100
#define D_TIME1_150MS  150
#define D_TIME1_1S		 1000
#define D_TIME1_P_OVERFLOW	12000
#define D_TIME1_I_OVERFLOW	8000

#define D_TIME1_CAL_TIME 20000			//1000w负载在36s内 耗电0.01度

static uint16_t U16_P_TotalTimes; 			/* 记录溢出的超时时间 */
static uint16_t U16_I_TotalTimes;

static uint16_t U16_P_OneCycleTime;     /* 功率测量时间   */
static uint16_t U16_I_OneCycleTime;

static uint16_t U16_P_LastOneCycleTime;  /* 功率测量时间，上一次的数值 */
static uint16_t U16_I_LastOneCycleTime;

static uint16_t U16_P_CNT = 0;								/* 脉冲数量 */
static uint16_t U16_I_CNT = 0;

static uint16_t U16_P_Last_CNT;					/* 脉冲数量，上一次数值 */
static uint16_t U16_I_Last_CNT;

static bool B_P_TestOneCycle_Mode;			/*功率测量模式，true单周期模式(被测周期大，100ms，记录一个周期的时间  */
static bool B_I_TestOneCycle_Mode;

static bool B_P_Last_TestOneCycle_Mode;																				/*              false 脉冲计数模式，计时1秒，看看有多少个脉冲  */
static bool B_I_Last_TestOneCycle_Mode;

static bool B_P_OVERFLOW; 						/*  脉冲计时溢出标志，表示周期时间太长，测量不到 */
static bool B_I_OVERFLOW;

static bool B_P_Last_OVERFLOW;
static bool B_I_Last_OVERFLOW;

volatile uint16_t U16_REF_001_E_Pluse_CNT; //0.01 参考值
static uint16_t U16_E_Pluse_CNT;		//脉冲个数计数
volatile uint32_t U32_E_Plus_5Min;		//5分钟内的充电电量脉冲计数
volatile uint32_t U32_AC_E;						//总电量 *0.01

volatile uint32_t mCurrentSubTotal = 0;

static uint16_t U16_AC_I;
volatile uint16_t U16_AC_P;

static uint32_t U32_P_REF_PLUSEWIDTH_TIME;
static uint32_t U32_I_REF_PLUSEWIDTH_TIME;
static uint32_t U32_P_CURRENT_PLUSWIDTH_TIME;
static uint32_t U32_I_CURRENT_PLUSWIDTH_TIME;

static uint16_t U16_P_REF_Data = 18000;
static uint16_t U16_I_REF_Data = 9000;

static uint32_t U32_Cal_Times;			//计时36秒

#define D_ERR_MODE 0x00
#define D_NORMAL_MODE  0x10
#define D_CAL_START_MODE 0x21
#define D_CAL_END_MODE 0x23

static uint8_t U8_CURR_WorkMode = D_NORMAL_MODE;

static void TIM3_INT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 燭IM3 中断嵌套设计*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
48000000/x = y Hz
1/y * z = 0.001

x * z = 48000
*/
static void TIM3_OUT_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure;
	uint16_t TM3_PrescalerValue = 0;
	uint16_t TM3_PRESCALER = 480;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	/* 计算预分频值 */
	TM3_PrescalerValue = 479;//(uint16_t) (SystemCoreClock/TM3_PRESCALER) - 1;

/* Time 定时器基础设置 */
	TIM3_TimeBaseStructure.TIM_Period = 100;
	TIM3_TimeBaseStructure.TIM_Prescaler = 0;
	TIM3_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);

	/* 预分频器配置 */
	TIM_PrescalerConfig(TIM3, TM3_PrescalerValue, TIM_PSCReloadMode_Immediate);
	
	/* TIM 中断使能 */
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);

	TIM3_INT_Config();
	/* TIM3 使能 */
	TIM_Cmd(TIM3, ENABLE);
}

/*
 current sample gpio PA15
  power sample gpio PA12
*/
static void init_sample_current_power_eint_irq(void)
{
	int i;
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */

	/* init every gpio */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  /* Connect EXTI Line to PXX pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);	
	
	/* Configure EXTI2 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#include "userflash.h"
void print_int(uint8_t *str, int val);
static void save_powerinfo_flash(void)
{
	UserNvdata tmpNv;
	tmpNv.magic = FLASH_USER_MAGIC;
	tmpNv.ADDR_REF_P_PLUSEWIDTH_TIME = U32_P_CURRENT_PLUSWIDTH_TIME;
	tmpNv.ADDR_REF_I_PLUSEWIDTH_TIME = U32_I_CURRENT_PLUSWIDTH_TIME;
	tmpNv.ADDR_REF_001_E = U16_REF_001_E_Pluse_CNT;
	tmpNv.ADDR_AC_BACKUP_E = 0;
	
	save_user_flash(&tmpNv);
	print_int("ref p=", tmpNv.ADDR_REF_P_PLUSEWIDTH_TIME);
	print_int("ref i=", tmpNv.ADDR_REF_I_PLUSEWIDTH_TIME);
	print_int("ref e=", tmpNv.ADDR_REF_001_E);
}

static void load_powerinfo_flash(void)
{
	UserNvdata *pNv = get_user_flash();
	U32_P_REF_PLUSEWIDTH_TIME = pNv->ADDR_REF_P_PLUSEWIDTH_TIME;
	U32_I_REF_PLUSEWIDTH_TIME = pNv->ADDR_REF_I_PLUSEWIDTH_TIME;
	U16_REF_001_E_Pluse_CNT = pNv->ADDR_REF_001_E;
	U32_AC_E = pNv->ADDR_AC_BACKUP_E;
}

static void update_powerinfo_flash(void)
{
	UserNvdata tmpNv;
	UserNvdata *pNv = get_user_flash();

	memcpy(&tmpNv, pNv, sizeof(tmpNv));
	
	tmpNv.ADDR_AC_BACKUP_E = U32_AC_E;
	save_user_flash(&tmpNv);
}

void user_call_calibrate_power(void)
{
	if (U8_CURR_WorkMode == D_NORMAL_MODE) {	//calibrate
		process_post(&process_hw_timer,  PROCESS_ACTION_CAL_PREPARE, NULL);
	}
}

#include "lock_pwr.h"
extern uint32_t getCurTicks(void);
extern struct process process_lock_pwr;
extern enum LOCK_STATUS mLockStatus;
#define CHARGET_CURRENT_DETECTED  150
#define CHARGET_CURRENT_FULL_DETECTED  120
#define CHARGET_CURRENT_NO_DEVICE		100
#define CHARGET_CURRENT_OVERLOAD  10000

static uint32_t mLastChargeFull = 0;
static uint32_t mLastNoPlugIn = 0;
volatile bool mInCalMode = false;
static void handle_current_callback(uint16_t  current)
{
	//print_int("current:" , current);
	//print_int("lockstat:" , mLockStatus);
	if (current > CHARGET_CURRENT_OVERLOAD){
		if (!mInCalMode) {
			process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_OVERLOAD, NULL);	
			print_string("overload");
		}
	}
}

void print_int(uint8_t *str, int val);

static struct etimer mCalPrepareEtimer;
//static uint16_t mNoLoadDetect[4];
//static uint8_t mNoLoadDetectIndex = 0;

bool user_call_unlock(void);
PROCESS_THREAD(process_hw_timer, ev, data)
{
	uint32_t a;
	
  PROCESS_BEGIN();
	
	TIM3_OUT_Config();
	init_sample_current_power_eint_irq();
	load_powerinfo_flash();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				if (!B_P_Last_OVERFLOW) {
					if (B_P_Last_TestOneCycle_Mode) {
						U32_P_CURRENT_PLUSWIDTH_TIME = U16_P_LastOneCycleTime *1000;
					} else {
						U32_P_CURRENT_PLUSWIDTH_TIME = ((uint32_t)U16_P_LastOneCycleTime * 1000)/(U16_P_Last_CNT-1);
					}
					//print_int((uint8_t*)"p width=", U32_P_CURRENT_PLUSWIDTH_TIME);
					a = U16_P_REF_Data * U32_P_REF_PLUSEWIDTH_TIME;
					if (U32_P_CURRENT_PLUSWIDTH_TIME != 0) {
						U16_AC_P = a / U32_P_CURRENT_PLUSWIDTH_TIME;
					} else {
						U16_AC_P = 0;
					}
					//print_int((uint8_t *)"p = ", U16_AC_P);
					if (U16_AC_P == 0xffff) {
						U16_AC_P = 0;
					}
				} else {
					U16_AC_P = 0;
				}
			//	print_int("pcnt=",U16_E_Pluse_CNT);
		//		print_int("ref pcnt", U16_REF_001_E_Pluse_CNT);
				//print_int((uint8_t *)"p = ", U16_AC_P);
				break;
			
			case PROCESS_EVENT_TIMER:
				//handle_led_timeout();
				if (data == &mCalPrepareEtimer) {
					print_string("cal start ....");
					U32_Cal_Times = 0;
					U16_E_Pluse_CNT = 0;
					U8_CURR_WorkMode = D_CAL_START_MODE;
				}
				break;
			
			case PROCESS_ACTION_CAL_END:
				//print_int("p ref width=", U32_P_CURRENT_PLUSWIDTH_TIME);
				save_powerinfo_flash();
				load_powerinfo_flash();
				U8_CURR_WorkMode = D_NORMAL_MODE;
				mInCalMode = false;
				print_string("cal end ....");
				process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_DISCHARGE_COMMAND, NULL);
				break;
			
			case PROCESS_ACTION_CAL_PREPARE:
				mInCalMode = true;
				if (user_call_unlock()) {
					//caculate prepare
					print_string("cal prepare ...");
					etimer_set(&mCalPrepareEtimer, CLOCK_SECOND);
				} else {
					mInCalMode = false;
					print_string("cal prepare failed");
				}
				break;
			
			case PROCESS_ACTION_CAL_START:
				break;
			
			case PROCESS_ACTION_I_SAMPLED:
				if (!B_I_Last_OVERFLOW) {
					if (B_I_Last_TestOneCycle_Mode) {		 //表示单周期模式，在该模式下，才有可能发生空载，记录4次的周期的数值
						U32_I_CURRENT_PLUSWIDTH_TIME = U16_I_LastOneCycleTime * 1000;
						//mNoLoadDetect[mNoLoadDetectIndex&0x03] = U16_I_LastOneCycleTime;
						//if (abs(mNoLoadDetect[0] - mNoLoadDetect[1])
					} else {
						U32_I_CURRENT_PLUSWIDTH_TIME = ((uint32_t)U16_I_LastOneCycleTime * 1000) / U16_I_Last_CNT;
					}
					
					//if (U32_I_CURRENT_PLUSWIDTH_TIME == 0) {
					//	print_int((uint8_t*)"time=", U16_I_LastOneCycleTime);
					//	print_int((uint8_t *)"cnt=", U16_I_Last_CNT);
					//}
					a = U16_I_REF_Data *U32_I_REF_PLUSEWIDTH_TIME;
					U16_AC_I = a / U32_I_CURRENT_PLUSWIDTH_TIME;
					if (U16_AC_I >180) {
					}
					
					if (U16_AC_P  == 0){
						U16_AC_I = 0;
					}
					if (U16_AC_I == 0xffff) {
						U16_AC_I = 0;
					}
				//	if (B_I_OVERFLOW == true) {
				//	}
				//	U16_AC_I = 0;
				} else {
					U16_AC_I = 0;
				}
				//print_int((uint8_t *)"i = ", U16_AC_I);
				//print_int((uint8_t*)"i width=", U32_I_CURRENT_PLUSWIDTH_TIME);
				handle_current_callback(U16_AC_I);
				break;
			
			case PROCESS_ACTION_NOSQUREWAVE_REPORT:
				print_string("detect non square wave");
				break;
			
			case PROCESS_ACTION_SQUREWAVE_REPORT:
				print_string("detect square wave");
				break;
			default:
				break;
		}
  }

  PROCESS_END();
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		if (U8_CURR_WorkMode == D_CAL_START_MODE) {
			U32_Cal_Times++;
			if (U32_Cal_Times >= D_TIME1_CAL_TIME) {
				//U8_CURR_WorkMode = D_CAL_END_MODE;
				U16_REF_001_E_Pluse_CNT = U16_E_Pluse_CNT;
				process_post(&process_hw_timer, PROCESS_ACTION_CAL_END, NULL);
			}
		}
		if (U16_P_CNT != 0) {//开始计时
			U16_P_OneCycleTime++;
			U16_P_TotalTimes++;
		}
		
		if (U16_P_TotalTimes >= D_TIME1_P_OVERFLOW) {		/* 周期太长，一个脉冲均没有收到 */
			B_P_OVERFLOW = true;
			B_P_Last_OVERFLOW = B_P_OVERFLOW;
			U16_P_TotalTimes = 0;
			U16_P_OneCycleTime = 0;
			U16_P_CNT = 0;
			B_P_TestOneCycle_Mode = 0;
		} else if (U16_P_OneCycleTime == D_TIME1_100MS) {
			//到达一个100ms的时间点
			if(U16_P_CNT < 2) {	//周期>100ms
				B_P_TestOneCycle_Mode = 1;
			} else {
				B_P_TestOneCycle_Mode = 0;
			}
		}
		
		//电流测量
		if (U16_I_CNT != 0) {
			U16_I_OneCycleTime++;
			U16_I_TotalTimes++;
		}
		if (U16_I_TotalTimes >= D_TIME1_I_OVERFLOW) {
			B_I_OVERFLOW = true;
			B_I_Last_OVERFLOW = B_I_OVERFLOW;
			
			U16_I_TotalTimes = 0;
			U16_I_OneCycleTime = 0;
			U16_I_CNT = 0;
			B_I_TestOneCycle_Mode = 0;
		} else if (U16_I_OneCycleTime == D_TIME1_100MS) {
			if (U16_I_CNT < 2) {
				B_I_TestOneCycle_Mode = true;
			} else {
				B_I_TestOneCycle_Mode = false;
			}
		}
	}
}


static uint16_t mLastTotalCount = 0;
static uint8_t mSquareWave=0;
static uint8_t mSquareWaveIndex = 0x00;
static bool isNowSquareWave = false;
bool isNowSquareWaveCheck(void)
{
	return isNowSquareWave;
}
void print_int2(uint8_t *str, int val, int val2);

volatile bool mNeedCheckSquareWave = false;

extern uint32_t getCurTicks(void);
static uint32_t mLastTicks = 0;
volatile uint8_t mNowSquareWaveCount = 0;

static void handle_current_IRQHandler(void)
{
	uint8_t i,sum;
	uint32_t curticks;
		//print_int2("m", mLastTotalCount, U16_I_TotalTimes);
		if (abs(mLastTotalCount - U16_I_TotalTimes) < 25) {		//2这时间相差50ms
			if (mNeedCheckSquareWave){
				mSquareWave |= (1 << (mSquareWaveIndex & 0x07));
				if (mSquareWave == 0xff) {
					curticks = getCurTicks();
					if ((curticks - mLastTicks) > 1000) {
						mNowSquareWaveCount++;
					
						mLastTicks = curticks;
						if (mNowSquareWaveCount > 2) {
							if (!isNowSquareWave) {
								isNowSquareWave = true;
								//process_post(&process_hw_timer, PROCESS_ACTION_SQUREWAVE_REPORT, NULL);
								process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_PLUGIN, NULL);
							}
						}
					}
				}	
			}
		} else {
			mSquareWave &= ~(1 << (mSquareWaveIndex & 0x07));
			mNowSquareWaveCount = 0;
			sum = 0;
			for (i = 0; i < 8; i++) {
				if (mSquareWave & (1 << i)) {
					sum++;
				}
			}
			if (sum < 3) {
				if (isNowSquareWave) {
					isNowSquareWave = false;
					//process_post(&process_hw_timer, PROCESS_ACTION_NOSQUREWAVE_REPORT, NULL);
					process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_PLUGOUT, NULL);
				}
			}	
		}
		mSquareWaveIndex++;
		mLastTotalCount = U16_I_TotalTimes;
		U16_I_TotalTimes = 0;
		U16_I_CNT++;
		if (B_I_OVERFLOW == true) {
			B_I_TestOneCycle_Mode = false;			//溢出模式转入，开始重新测试，初始化为脉冲计数模式
			U16_I_TotalTimes = 0;
			U16_I_OneCycleTime = 0;
			U16_I_CNT = 1;
			B_I_OVERFLOW = false;
		} else {
			if (B_I_TestOneCycle_Mode) {
				if (U16_I_OneCycleTime >= D_TIME1_100MS) {
					U16_I_LastOneCycleTime = U16_I_OneCycleTime;
					B_I_Last_TestOneCycle_Mode = B_I_TestOneCycle_Mode;
					B_I_OVERFLOW = false;
					B_I_Last_OVERFLOW = B_I_OVERFLOW;
					/* 准备下次测试 */
					B_I_TestOneCycle_Mode = 0;
					U16_I_TotalTimes = 0;
					U16_I_OneCycleTime = 0;
					U16_I_CNT = 1;
					process_post(&process_hw_timer, PROCESS_ACTION_I_SAMPLED, NULL);
				}
			} else {
				if (U16_I_OneCycleTime >= D_TIME1_1S) {
					U16_I_LastOneCycleTime = U16_I_OneCycleTime;
					U16_I_Last_CNT = U16_I_CNT;
					B_I_Last_TestOneCycle_Mode = B_I_TestOneCycle_Mode;
					B_I_OVERFLOW = false;
					B_I_Last_OVERFLOW = B_I_OVERFLOW;
					
					//准备下次测量
					B_I_TestOneCycle_Mode = 0;
					U16_I_TotalTimes = 0;
					U16_I_OneCycleTime = 0;
					U16_I_CNT = 1;
					process_post(&process_hw_timer, PROCESS_ACTION_I_SAMPLED, NULL);
				}
			}
		}
		

		return;
}

void handle_power_IRQHandler(void)
{
		
		U16_P_TotalTimes = 0;				/* 溢出清零  */
		U16_P_CNT++;
		if (B_P_OVERFLOW) {
			B_P_TestOneCycle_Mode = false;  /*计数脉冲操作 */
			U16_P_TotalTimes = 0;				
			U16_P_OneCycleTime = 0;
			U16_P_CNT = 1;							/* 开始计时 */
			B_P_OVERFLOW = false;
		} else {
			if (B_P_TestOneCycle_Mode) {		//单周期测量，进入单周期测量的时候，已经发生了100ms事件，并且该事件下没有检测到第二个下降沿，也就本周期还没有结束*/
																			//进入这里刚好说明发生了第二次下降沿，也即一个周期
				if (U16_P_OneCycleTime >= D_TIME1_100MS) {			
					U16_P_LastOneCycleTime = U16_P_OneCycleTime;
					B_P_Last_TestOneCycle_Mode = B_P_TestOneCycle_Mode;
					B_P_Last_OVERFLOW = B_P_OVERFLOW;
					B_P_TestOneCycle_Mode = false;			/* 进入计数脉冲 */
					U16_P_CNT = 1;
					U16_P_TotalTimes = 0;
					U16_P_OneCycleTime = 0;
					process_poll(&process_hw_timer);
				}
			} else {
				if (U16_P_OneCycleTime >= D_TIME1_1S) {
					U16_P_LastOneCycleTime = U16_P_OneCycleTime;
					U16_P_Last_CNT = U16_P_CNT;
					B_P_Last_TestOneCycle_Mode = B_P_TestOneCycle_Mode;
					B_P_Last_OVERFLOW = B_P_OVERFLOW;
					B_P_TestOneCycle_Mode = false;
					U16_P_TotalTimes = 0;
					U16_P_OneCycleTime = 0;
					U16_P_CNT = 1;
					process_poll(&process_hw_timer);
				}
			}
		}
		
		if (U8_CURR_WorkMode == D_CAL_START_MODE) {
			U16_E_Pluse_CNT++;
		}
		if (U8_CURR_WorkMode == D_NORMAL_MODE) {
			U16_E_Pluse_CNT++;
			U32_E_Plus_5Min++;
			if (U16_E_Pluse_CNT >= U16_REF_001_E_Pluse_CNT) {
				U16_E_Pluse_CNT = 0;
				U32_AC_E++;
				mCurrentSubTotal++;
				//process_post(&process_hw_timer, PROCESS_ACTION_UPDATE_POWER, NULL);
			}
		}
		
		return;
}

extern struct process process_leakage;
extern struct process process_gpio_key;
void EXTI4_15_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line15) != RESET) { 
		EXTI_ClearITPendingBit(EXTI_Line15);
		handle_power_IRQHandler();
	} else if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line12);
		handle_current_IRQHandler();
	} else if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line4);
		//print_string("irq leakage");
		process_poll(&process_leakage);
	} else if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line5);
		process_poll(&process_gpio_key);
	} else {
		EXTI_ClearITPendingBit(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15);
	}
}



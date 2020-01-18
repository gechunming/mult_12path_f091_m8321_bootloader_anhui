#include "stm32f0xx.h"
#include "contiki.h"
#include <stdint.h>
#include <stdbool.h>
#include "user_usart1.h"
#include "stdio.h"
#include "lock_pwr.h"
#include "lock_report.h"
#include "led2.h"

PROCESS(process_lock_pwr, "lock pwr process");

static struct etimer mEtimer;
static struct etimer mPlugInCheckTimer;
//static struct etimer mChargedEndEtimer;

static uint32_t mChargedCurrentTimeM = 0;			/* 当前累计充电时间  */
static uint32_t mMaxChargeTimeM = 8 * 60;			/* 最大充电8个小时 */

extern volatile bool mNeedCheckSquareWave;
enum LOCK_STATUS mLockStatus = LOCK_STATUS_CLOSED;

static uint8_t mChargedFullReport = 0;

extern volatile uint32_t U32_AC_E;						//总电量 *0.01
extern volatile uint16_t U16_AC_P;
extern volatile uint32_t mCurrentSubTotal;

/*
PA11 enable power
*/
void poweronoff_charger(bool enable);

bool isLockPowerOff(void)
{
	if (mLockStatus == LOCK_STATUS_CLOSED) {
		return true;
	} else {
		return false;
	}
}

bool isLockCharging(void)
{
	if ((mLockStatus == LOCK_STATUS_OPENED) || (mLockStatus == LOCK_STATUS_CHARGING)) {
		return true;
	} else {
		return false;
	}
}

bool isLockPowerFull(void)
{
	if (mLockStatus == LOCK_STATUS_CHARGED) {
		return true;
	} else {
		return false;
	}
}

static void lock_pwr_gpio_init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */

	/* init every gpio */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	poweronoff_charger(false);
}

void poweronoff_charger(bool enable)
{
	if (enable) {
		GPIO_SetBits(GPIOA, GPIO_Pin_11);
	} else {
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
		mNeedCheckSquareWave = false;
	}
}
extern volatile uint8_t mNowSquareWaveCount;
bool isLeakageNow(void);
bool isTemperatureHighNow(void);
bool isNowSquareWaveCheck(void);
bool user_call_unlock(void)
{
	if (mLockStatus != LOCK_STATUS_CLOSED) {
		print_int((uint8_t*)"unlock failed curstate=", mLockStatus);
		return false;
	}
	if (isLeakageNow() || isTemperatureHighNow()) {
		print_string("leakage or temperature check failed");
		return false;
	}
	mLockStatus = LOCK_STATUS_OPENED;
	poweronoff_charger(true);
	led_status_set(LED_STATUS_BLINK_BLUE);
	print_string("unlock");
	
	PROCESS_CONTEXT_BEGIN(&process_lock_pwr);
	etimer_set(&mEtimer, 35 * CLOCK_SECOND);
	etimer_set(&mPlugInCheckTimer, CLOCK_SECOND);
	PROCESS_CONTEXT_END(&process_lock_pwr);
	mCurrentSubTotal = 0;
	mChargedCurrentTimeM = 0;
	mNowSquareWaveCount = 0;
	if (isNowSquareWaveCheck()) {
				print_string("already plugin ");
		process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_PLUGIN, NULL);
	}
	
	return true;
}

bool unlock_after_reboot(uint32_t curpower)
{
	if (mLockStatus != LOCK_STATUS_CLOSED) {
		print_int((uint8_t*)"unlock failed curstate=", mLockStatus);
		return false;
	}
	if (isLeakageNow() || isTemperatureHighNow()) {
		print_string("leakage or temperature check failed");
		return false;
	}
	mLockStatus = LOCK_STATUS_OPENED;
	poweronoff_charger(true);
	led_status_set(LED_STATUS_BLINK_BLUE);
	print_string("unlock by reboot");
	
	PROCESS_CONTEXT_BEGIN(&process_lock_pwr);
	etimer_set(&mEtimer, 35 * CLOCK_SECOND);
	etimer_set(&mPlugInCheckTimer, CLOCK_SECOND);
	PROCESS_CONTEXT_END(&process_lock_pwr);
	mCurrentSubTotal = curpower;
	mChargedCurrentTimeM = 0;
	mNowSquareWaveCount = 0;
	
	if (isNowSquareWaveCheck()) {
		print_string("already plugin ");
		process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_PLUGIN, NULL);
	}
	
	return true;
}


void user_lock_charger_full_report(void);

void user_lock_report(enum LOCK_REASON reson, uint32_t subtotal, uint32_t total);
void user_power_report(uint32_t subtotal, uint32_t power);
/*
开锁，还未检测到用户插入设备充电的场景 
*/
extern volatile uint16_t U16_REF_001_E_Pluse_CNT; //0.01 参考值
static uint32_t mCur_U32_E_Plus_5Min_Max = 0x00;
extern volatile uint32_t U32_E_Plus_5Min;		//5分钟内的充电电量脉冲计数
static void handle_action_when_opened(process_event_t ev)
{
	switch(ev) {
		case LOCK_PROCESS_ACTION_PLUGIN:
			mChargedFullReport = 0;
			U32_E_Plus_5Min = 0;
			mCur_U32_E_Plus_5Min_Max = 0;
			mLockStatus = LOCK_STATUS_CHARGING;
			etimer_set(&mEtimer, CLOCK_SECOND * 60 * 5);
			print_string("now plug in ..");
			break;
		
		case LOCK_PROCESS_ACTION_CHARGEFULL:
			user_lock_charger_full_report();
			mLockStatus = LOCK_STATUS_CHARGED;
			etimer_set(&mEtimer, CLOCK_SECOND * 60);			/* 2 hours will power off */
			mChargedFullReport = 1;
			led_status_set(LED_STATUS_BLINK_RED);
			break;
		
		case LOCK_PROCESS_ACTION_PLUGOUT:				/* 用户拔出 */
			user_lock_report(LOCK_REASON_PLUGOUT, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_DISCHARGE_COMMAND:			/*  用户tcp命令关闭 */
			user_lock_report(LOCK_REASON_LOCKCMD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_TEMPERATURE_WARNING:
			user_lock_report(LOCK_REASON_TEMPERATURE_WARNING, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case LOCK_PROCESS_ACTION_OVERLOAD:
			user_lock_report(LOCK_REASON_OVERLOAD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			refresh_led_status();
			break;
		
		case LOCK_PROGESS_ACTION_LEAKAGE:
			user_lock_report(LOCK_REASON_LEAKAGE, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case PROCESS_EVENT_TIMER:							/* 超时 没有检测到 插入设备充电 */
			user_lock_report(LOCK_REASON_NOCHARGE, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_BLINK_TWO);
			break;
		
		default:
			break;
	}
}

/*
开锁，并且检测到了在充电的场景 
*/



static void handle_action_when_charging(process_event_t ev)
{
	switch(ev) {
		case LOCK_PROCESS_ACTION_PLUGIN:
			mChargedFullReport = 0;
			mLockStatus = LOCK_STATUS_CHARGING;
			etimer_set(&mEtimer, CLOCK_SECOND * 60 * 5);
			break;
		
		case LOCK_PROCESS_ACTION_CHARGEFULL:
			print_string("send charger full");
			user_lock_charger_full_report();
			mLockStatus = LOCK_STATUS_CHARGED;
			etimer_set(&mEtimer, CLOCK_SECOND * 60);			/* 2 hours will power off */
			mChargedFullReport = 1;
			led_status_set(LED_STATUS_BLINK_RED);
			break;
		
		case LOCK_PROCESS_ACTION_PLUGOUT:				/* 用户拔出 */
			user_lock_report(LOCK_REASON_PLUGOUT, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_DISCHARGE_COMMAND:			/*  用户tcp命令关闭 */
			user_lock_report(LOCK_REASON_LOCKCMD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_TEMPERATURE_WARNING:
			user_lock_report(LOCK_REASON_TEMPERATURE_WARNING, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case LOCK_PROCESS_ACTION_OVERLOAD:
			user_lock_report(LOCK_REASON_OVERLOAD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			refresh_led_status();
			break;
		
		case LOCK_PROGESS_ACTION_LEAKAGE:
			user_lock_report(LOCK_REASON_LEAKAGE, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case PROCESS_EVENT_TIMER:							/* 超时 没有检测到 插入设备充电 */
			etimer_set(&mEtimer, CLOCK_SECOND * 5 *60);
			
			if (U32_E_Plus_5Min > mCur_U32_E_Plus_5Min_Max) {
				mCur_U32_E_Plus_5Min_Max = U32_E_Plus_5Min;
			}
			if ((U32_E_Plus_5Min < (U16_REF_001_E_Pluse_CNT>>3)) || //小于15W
					(U32_E_Plus_5Min < (mCur_U32_E_Plus_5Min_Max/5))) {  //小于最大的1/5
				process_post(&process_lock_pwr, LOCK_PROCESS_ACTION_CHARGEFULL, NULL);
				print_string("charge full post");
			}
			mChargedCurrentTimeM += 5;
			if (mChargedCurrentTimeM > mMaxChargeTimeM) {
				user_lock_report(LOCK_REASON_LOCKCMD, mCurrentSubTotal, U32_AC_E);
				poweronoff_charger(false);
				mLockStatus = LOCK_STATUS_CLOSED;
				//led_status_set(LED_STATUS_BLINK_TWO);
				refresh_led_status();
				break;
			}
			user_power_report(mCurrentSubTotal, U16_AC_P/10);
			U32_E_Plus_5Min = 0;
			break;
		
		default:
			break;
	}
}

/*
开锁，并且检测到充满的场景 
*/
static void handle_action_when_charged(process_event_t ev)
{
	switch(ev) {
		case LOCK_PROCESS_ACTION_PLUGIN:
			mLockStatus = LOCK_STATUS_CHARGING;
			etimer_set(&mEtimer, CLOCK_SECOND * 60 * 5);
			break;
		
		case LOCK_PROCESS_ACTION_CHARGEFULL:
			user_lock_charger_full_report();
			mLockStatus = LOCK_STATUS_CHARGED;
			etimer_set(&mEtimer, CLOCK_SECOND * 60);			/* 2 hours will power off */
			mChargedFullReport = 1;
			led_status_set(LED_STATUS_BLINK_RED);
			//refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_PLUGOUT:				/* 用户拔出 */
			user_lock_report(LOCK_REASON_PLUGOUT, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_DISCHARGE_COMMAND:			/*  用户tcp命令关闭 */
			user_lock_report(LOCK_REASON_LOCKCMD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			//led_status_set(LED_STATUS_BLINK_TWO);
			refresh_led_status();
			break;
		
		case LOCK_PROCESS_ACTION_TEMPERATURE_WARNING:
			user_lock_report(LOCK_REASON_TEMPERATURE_WARNING, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case LOCK_PROCESS_ACTION_OVERLOAD:
			user_lock_report(LOCK_REASON_OVERLOAD, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
			refresh_led_status();
			break;
		
		case LOCK_PROGESS_ACTION_LEAKAGE:
			user_lock_report(LOCK_REASON_LEAKAGE, mCurrentSubTotal, U32_AC_E);
			poweronoff_charger(false);
			mLockStatus = LOCK_STATUS_CLOSED;
		led_status_set(LED_STATUS_ALWAYS_ON_RED);
			break;
		
		case PROCESS_EVENT_TIMER:						
			//report charger full
			print_int("charget full times", mChargedFullReport);
			if (mChargedFullReport < 3) {
				user_lock_charger_full_report();
				mLockStatus = LOCK_STATUS_CHARGED;
				etimer_set(&mEtimer, CLOCK_SECOND * 60);			/* 2 hours will power off */
				mChargedFullReport++;
				mChargedCurrentTimeM += 1;
			} else if (mChargedFullReport == 3) {
				mLockStatus = LOCK_STATUS_CHARGED;
				etimer_set(&mEtimer, CLOCK_SECOND * 60 * 5);
				mChargedFullReport++;
				mChargedCurrentTimeM += 1;
			} else {
				mChargedFullReport++;
				mChargedCurrentTimeM += 5;
				if ((mChargedFullReport > 26) || (mChargedCurrentTimeM > mMaxChargeTimeM)) {
					user_lock_report(LOCK_REASON_LOCKCMD, mCurrentSubTotal, U32_AC_E);
					mLockStatus = LOCK_STATUS_CLOSED;
					//report charger off
					poweronoff_charger(false);
					//led_status_set(LED_STATUS_BLINK_TWO);
					refresh_led_status();
					break;
				}
				user_power_report(mCurrentSubTotal, U16_AC_P/10);
				etimer_set(&mEtimer, CLOCK_SECOND * 60 * 5);
			}
			break;
		
		default:
			break;
	}
}
static void handle_user_event(process_event_t ev)
{
		switch(mLockStatus) {
			case LOCK_STATUS_CLOSED:				/*  only process  user open lock command */
				break;
			
			case LOCK_STATUS_OPENED:			
				handle_action_when_opened(ev);
				break;
			
			case LOCK_STATUS_CHARGING:
				handle_action_when_charging(ev);
				break;
			
			case LOCK_STATUS_CHARGED:
				handle_action_when_charged(ev);
				break;
			
			default:
				break;
		}
}

PROCESS_THREAD(process_lock_pwr, ev, data)
{

  PROCESS_BEGIN();
	
	lock_pwr_gpio_init();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
				if (data == &mPlugInCheckTimer) {
					mNeedCheckSquareWave = true;
					break;
				}
			default:
				handle_user_event(ev);
				break;
		}
  }

  PROCESS_END();
}



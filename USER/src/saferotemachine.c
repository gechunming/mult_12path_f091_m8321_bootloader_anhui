#include "stm32f0xx.h"
#include "contiki.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "tcp_protocal.h"
#include <stdio.h>
#include "bmp180.h"
#include "cablelock.h"
#include "led.h"
#include "saferopemachine.h"
#include "tcp_protocal.h"

static SafemachineStatus gSafeStatus = SAFE_MACHINE_STATUS_INIT;

PROCESS(process_safemachine, "safemachine process");
static struct etimer gSafeMachineTimer;

extern struct process process_bmp180;
extern struct process process_cable_lock;

#define SAFEMACHINE_NO_VIBRATE_IN_AIR_TIMEOUT (CLOCK_SECOND * 15 * 60)
#define SAFEMACHINE_NO_LOCKED_IN_AIR_TIMEOUT (CLOCK_SECOND * 15 * 60)

static int safemachine_send_nb_package(int8_t cmd, int8_t value)
{
	int8_t buf[9];
	
	memset((void*)buf, ' ', sizeof(buf));
	buf[0] = value;
	return TCP_protocal_send(cmd, sizeof(buf), (uint8_t*)buf, false);
}

static uint32_t gLastVibrateTime = 0x00;
uint32_t getCurTicks(void);
uint32_t getTickTimePassed(uint32_t oldtime);
#define REFRESH_VIBRATE_TIME_INTERVAL_LOWMODE   (1000 * 20 * 3)			//will reset H0, after 20 minutes no vibrate

static void safemachine_init(void)
{
	if (getCableLockModeAlive() == CABLE_LOCK_MODE_LOCKED) { //in locked mode
		if (IsInLowMode()) {
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKLOW;
			gLastVibrateTime = getCurTicks();
			led_set(1, 0);
		} else {
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKHIGH;
			led_set(1, 1);
		}
	//	led_set(1,1);
	} else {
		if (IsInLowMode()) {
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKLOW;
			gLastVibrateTime = getCurTicks();
			led_set(1, 0);
		} else {
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKHIGH;
			led_set(1, 1);
		}
	//	led_set(1, 0);
	}
	start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT);
	
	PROCESS_CONTEXT_BEGIN(&process_safemachine);
	etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 20);
	PROCESS_CONTEXT_END(&process_safemachine);
}

static void safemachine_locklowmode_func(SafeMachineEvent event)
{
	switch(event) {
		case SAFE_MACHINE_EVENT_UNLOCKED:		/* locked on the ground */
			gLastVibrateTime = getCurTicks();
			led_blink(2, 5, 200, 200);
		//	led_set(1, 0);
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKLOW;
			start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT);
			etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 30);
		  safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_LOCKSTATUS_CMD, SAFEMACHINE_NB_UPLINK_LOCKSTATUS_VALUE_UNLOCKED);
			break;
		
		case SAFE_MACHINE_EVENT_TOHIGH:			/* lock mode, now is rising to sky */
			led_set(1,1);
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKHIGH;
			start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT | ALTITUDE_ACTION_SPEED_DETECT);
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_VIBRATE_IN_AIR_TIMEOUT);
			break;
		
		case SAFE_MACHINE_EVENT_REQUESTRESETH0:
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_REQUEST_CMD, SAFEMACHINE_NB_UPLINK_REQUEST_RESETH0);
			break;
		
		case SAFE_MACHINE_EVENT_NBREQUESTRESETH0:
			process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_KEYPRESSED:	/* unlock lock */
			//only run unlock function, to unlock the rope
			//reset h0
			led_blink(1, 3, 200, 200);
		  process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
		//  process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_VIBRATE:
			if (getTickTimePassed(gLastVibrateTime) > REFRESH_VIBRATE_TIME_INTERVAL_LOWMODE) {
				start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT | ALTITUDE_ACTION_H0_SET);
			} else {
				start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT);
			}
			print_string("start get altitut locklow\r\n");
		  etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 20);
			gLastVibrateTime = getCurTicks();
			break;
		
		case PROCESS_EVENT_TIMER:
			stop_measure_altitude();
		print_string("stop get altitut locklow\r\n");
			break;
		
		default:
			break;
	}
	return;
}

static void safemachine_unlocklowmode_func(SafeMachineEvent event)
{
	switch(event) {
		case SAFE_MACHINE_EVENT_LOCKED:		/* unlock on the ground */
			gLastVibrateTime = getCurTicks();
			led_blink(2, 5, 200, 200);
		//	led_set(1, 1);
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKLOW;
			start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT);
			print_string("start measure, unlock get lock\r\n");
			etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 30);
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_LOCKSTATUS_CMD, SAFEMACHINE_NB_UPLINK_LOCKSTATUS_VALUE_LOCKED);
			break;
		
		case SAFE_MACHINE_EVENT_TOHIGH:			/* unlock mode, but now is rising to sky */
			led_set(1, 1);
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKHIGH;
			start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT | ALTITUDE_ACTION_SPEED_DETECT);
			print_string("start get altitude to unlock to high\r\n");
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_LOCKED_IN_AIR_TIMEOUT);
			process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
		  //safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_WARNING_CMD, SAFEMACHINE_NB_UPLINK_WARNING_VALUE_UNLOCK_INAIR);
			//lock rope
		  //report warning
			//beep 3 times
			break;
		
		case SAFE_MACHINE_EVENT_REQUESTRESETH0:
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_REQUEST_CMD, SAFEMACHINE_NB_UPLINK_REQUEST_RESETH0);
			break;
		
		case SAFE_MACHINE_EVENT_NBREQUESTRESETH0:
			process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_KEYPRESSED:	/* unlock lock */
			//only run lock function, to lock the rope
			//reset h0
			led_blink(1, 3, 200, 200);
			process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
		  //process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_VIBRATE:
			if (getTickTimePassed(gLastVibrateTime) > REFRESH_VIBRATE_TIME_INTERVAL_LOWMODE) {
				start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT | ALTITUDE_ACTION_H0_SET);
			} else {
				start_measure_altitude(ALTITUDE_ACTION_HIGH_DETECT | ALTITUDE_ACTION_LOWER_DETECT);
			}
			print_string("start get altitude unlocklow\r\n");
		  etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 20);
			gLastVibrateTime = getCurTicks();
			break;
		
		case PROCESS_EVENT_TIMER:
			print_string("stop get altitude\r\n");
			stop_measure_altitude();
			break;
		
		default:
			break;
	}
	return;
}

static void safemachine_lockhighmode_func(SafeMachineEvent event)
{
	switch(event) {
		case SAFE_MACHINE_EVENT_UNLOCKED:		/* unlock in the air */
			led_blink(2, 5, 200, 200);
		//  led_set(1, 0);
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKHIGH;
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_LOCKSTATUS_CMD, SAFEMACHINE_NB_UPLINK_LOCKSTATUS_VALUE_UNLOCKED);
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_LOCKED_IN_AIR_TIMEOUT);
			break;
		
		case SAFE_MACHINE_EVENT_TOLOW:			/* lock mode, now is falling to ground */
			led_set(1, 0);
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKLOW;
			gLastVibrateTime = getCurTicks();
			etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 30);
			process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_KEYPRESSED:	/* unlock lock */
			//request unlock the rope
			led_blink(1, 3, 200, 200);
		  safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_REQUEST_CMD, SAFEMACHINE_NB_UPLINK_REQUEST_UNLOCK);
			break;
		
		case SAFE_MACHINE_EVENT_REQUESTUNLOCK:
			//network unlock call
			process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_REQUESTRESETH0:
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_REQUEST_CMD, SAFEMACHINE_NB_UPLINK_REQUEST_RESETH0);
			break;
		
		case SAFE_MACHINE_EVENT_NBREQUESTRESETH0:
			process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_DROP:
			//drop devent, report alarm
			//safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_WARNING_CMD, SAFEMACHINE_NB_UPLINK_WARNING_VALUE_DROP);
			break;
		
		case SAFE_MACHINE_EVENT_VIBRATE:
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_VIBRATE_IN_AIR_TIMEOUT);
			break;
		
		case PROCESS_EVENT_TIMER:
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_WARNING_CMD, SAFEMACHINE_NB_UPLINK_WARNING_VALUE_NO_WORKING);
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_VIBRATE_IN_AIR_TIMEOUT);
			break;
		default:
			break;
	}
	return;
}

static void safemachine_unlockhighmode_func(SafeMachineEvent event)
{
	switch(event) {
		case SAFE_MACHINE_EVENT_LOCKED:		/* locked in the air */
			led_blink(2, 5, 200, 200);
	//		led_set(1,1);
			gSafeStatus = SAFE_MACHINE_STATUS_LOCKHIGH;
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_LOCKSTATUS_CMD, SAFEMACHINE_NB_UPLINK_LOCKSTATUS_VALUE_LOCKED);
		etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_VIBRATE_IN_AIR_TIMEOUT);
			break;
		
		case SAFE_MACHINE_EVENT_TOLOW:			/* lock mode, now is falling to ground */
			led_set(1, 0);
			gLastVibrateTime = getCurTicks();
			gSafeStatus = SAFE_MACHINE_STATUS_UNLOCKLOW;
			etimer_set(&gSafeMachineTimer, CLOCK_SECOND * 30);
			break;
		
		case SAFE_MACHINE_EVENT_KEYPRESSED:	/* unlock lock */
			//lock rope
			led_blink(2, 3, 200, 200);
			led_blink(1, 3, 200, 200);
			process_post(&process_cable_lock, PROCESS_CABLE_EVENT_UNLOCK, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_REQUESTRESETH0:
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_REQUEST_CMD, SAFEMACHINE_NB_UPLINK_REQUEST_RESETH0);
			break;
		
		case SAFE_MACHINE_EVENT_NBREQUESTRESETH0:
			process_post(&process_bmp180, BMP180_PROCESS_EVENT_RESETH0, NULL);
			break;
		
		case SAFE_MACHINE_EVENT_DROP:
			//drop devent, report alarm
			//safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_WARNING_CMD, SAFEMACHINE_NB_UPLINK_WARNING_VALUE_DROP);
			break;
		
		case PROCESS_EVENT_TIMER:			//will trigger if still unlocked
			safemachine_send_nb_package(SAFEMACHINE_NB_UPLINK_WARNING_CMD, SAFEMACHINE_NB_UPLINK_WARNING_VALUE_UNLOCK_INAIR);
			etimer_set(&gSafeMachineTimer, SAFEMACHINE_NO_LOCKED_IN_AIR_TIMEOUT);
			break;
		
		default:
			break;
	}
	return;
}

static void safemachine_run_event(SafeMachineEvent event)
{
	switch(gSafeStatus) {
		case SAFE_MACHINE_STATUS_LOCKLOW:
			safemachine_locklowmode_func(event);
			break;
		case SAFE_MACHINE_STATUS_UNLOCKLOW:
			safemachine_unlocklowmode_func(event);
			break;
		case SAFE_MACHINE_STATUS_LOCKHIGH:
			safemachine_lockhighmode_func(event);
			break;
		case SAFE_MACHINE_STATUS_UNLOCKHIGH:
			safemachine_unlockhighmode_func(event);
			break;
		
		case SAFE_MACHINE_STATUS_INIT:
			safemachine_init();
		  break;
		
		default:
			safemachine_init();
			break;
	}
}

PROCESS_THREAD(process_safemachine, ev, data)
{

  PROCESS_BEGIN();
	
	safemachine_init();
	
  while(1) { 
    PROCESS_WAIT_EVENT();
		switch(ev) {
			case PROCESS_EVENT_POLL: //key press detect
				break;
			
			case PROCESS_EVENT_TIMER:
			case SAFE_MACHINE_EVENT_LOCKED:
			case SAFE_MACHINE_EVENT_UNLOCKED:
			case SAFE_MACHINE_EVENT_TOHIGH:
			case SAFE_MACHINE_EVENT_TOLOW:
			case SAFE_MACHINE_EVENT_KEYPRESSED:
			case SAFE_MACHINE_EVENT_REQUESTUNLOCK:
			case SAFE_MACHINE_EVENT_DROP:
			case SAFE_MACHINE_EVENT_VIBRATE:
			case SAFE_MACHINE_EVENT_NBREQUESTRESETH0:
			case SAFE_MACHINE_EVENT_REQUESTRESETH0:
				safemachine_run_event(ev);
				break;
			
			default:
				break;
		}
  }

  PROCESS_END();
}


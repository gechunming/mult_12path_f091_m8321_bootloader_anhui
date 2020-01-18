#include "contiki.h"
#include <stdint.h>
#include <rtc-board.h>

extern volatile uint32_t gTimerCur;
TimerTime_t RtcGetTimerValue( void );
clock_time_t clock_time(void)
{
 	TimerTime_t curTime = RtcGetTimerValue();
return curTime;
}

unsigned long clock_seconds(void)
{
	return RtcGetTimerValue()/CLOCK_CONF_SECOND;
}


#include "user_delay.h"

uint32_t getTickTimePassed(uint32_t oldtime);
extern uint32_t getCurTicks(void);
void DelayMs(uint32_t nTime)
{
		__IO uint32_t prevTime = getCurTicks() + nTime;
		
		while( prevTime >= getCurTicks()) {
		}
}

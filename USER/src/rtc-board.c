#include "rtc-board.h"

/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_DURATION                     0.48828125      // 1 tick every 488us
#define RTC_ALARM_TICK_PER_MS                       2.048           // 1/2.048 = tick duration in ms

/*!
 * Maximum number of days that can be handled by the RTC alarm counter before overflow.
 */
#define RTC_ALARM_MAX_NUMBER_OF_DAYS                28

/*!
 * Number of seconds in a minute
 */
static const uint8_t SecondsInMinute = 60;

/*!
 * Number of seconds in an hour
 */
static const uint16_t SecondsInHour = 3600;

/*!
 * Number of seconds in a day
 */
static const uint32_t SecondsInDay = 86400;

/*!
 * Number of hours in a day
 */
static const uint8_t HoursInDay = 24;

/*!
 * Number of seconds in a leap year
 */
static const uint32_t SecondsInLeapYear = 31622400;

/*!
 * Number of seconds in a year
 */
static const uint32_t SecondsInYear = 31536000;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Holds the current century for real time computation
 */
static uint16_t Century = 0;

/*!
 * Flag used to indicates a Calendar Roll Over is about to happen
 */
static bool CalendarRollOverReady = false;

/*!
 * Flag used to indicates a the MCU has waken-up from an external IRQ
 */
volatile bool NonScheduledWakeUp = false;

/*!
 * RTC timer context
 */
typedef struct RtcCalendar_s
{
    uint16_t CalendarCentury;     //! Keep track of century value
    RTC_DateTypeDef CalendarDate; //! Reference time in calendar format
    RTC_TimeTypeDef CalendarTime; //! Reference date in calendar format
} RtcCalendar_t;

/*!
 * Current RTC timer context
 */
RtcCalendar_t RtcCalendarContext;

/*!
 * \brief Flag to indicate if the timestamps until the next event is long enough
 * to set the MCU into low power mode
 */
static bool RtcTimerEventAllowsLowPower = false;

/*!
 * \brief Flag to disable the LowPower Mode even if the timestamps until the
 * next event is long enough to allow Low Power mode
 */
static bool LowPowerDisableDuringTask = false;


/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
//static bool WakeUpTimeInitialized = false;

/*!
 * \brief Hold the Wake-up time duration in ms
 */
volatile uint32_t McuWakeUpTime = 0;

/*!
 * \brief Hold the cumulated error in micro-second to compensate the timing errors
 */
static int32_t TimeoutValueError = 0;

/*!
 * \brief RTC wakeup time computation
 */
//static void RtcComputeWakeUpTime( void );

/*!
 * \brief Start the RTC Alarm (timeoutValue is in ms)
 */
static void RtcStartWakeUpAlarm( uint32_t timeoutValue );

/*!
 * \brief Converts a TimerTime_t value into RtcCalendar_t value
 *
 * \param[IN] timeCounter Value to convert to RTC calendar
 * \retval rtcCalendar New RTC calendar value
 */
//
// REMARK: Removed function static attribute in order to suppress
//         "#177-D function was declared but never referenced" warning.
// static RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
//
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter );

/*!
 * \brief Converts a RtcCalendar_t value into TimerTime_t value
 *
 * \param[IN/OUT] calendar Calendar value to be converted
 *                         [NULL: compute from "now",
 *                          Others: compute from given calendar value]
 * \retval timerTime New TimerTime_t value
 */
static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar );

/*!
 * \brief Converts a TimerTime_t value into a value for the RTC Alarm
 *
 * \param[IN] timeCounter Value in ms to convert into a calendar alarm date
 * \param[IN] now Current RTC calendar context
 * \retval rtcCalendar Value for the RTC Alarm
 */
static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( TimerTime_t timeCounter, RtcCalendar_t now );

/*!
 * \brief Returns the internal RTC Calendar and check for RTC overflow
 *
 * \retval calendar RTC calendar
 */
static RtcCalendar_t RtcGetCalendar( void );

/*!
 * \brief Check the status for the calendar year to increase the value of Century at the overflow of the RTC
 *
 * \param[IN] year Calendar current year
 */
static void RtcCheckCalendarRollOver( uint8_t year );

#define RTC_CLOCK_SOURCE_LSI 1			//使用内部40k低频晶体

/*
enable rtc clock
*/
static void RTC_Config(void)
{
	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);

#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
	/* The RTC Clock may varies due to LSI frequency dispersion. */   
	/* Enable the LSI OSC */ 
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
#endif /* RTC_CLOCK_SOURCE_LSI */

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();
}

static int RTC_Set_DateTime(void)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;

	RTC_DateStructure.RTC_Year = 0;
	RTC_DateStructure.RTC_Month = 1;
	RTC_DateStructure.RTC_Date = 1;
	RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Saturday;
	if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
	{
		return -1;
	} 
	
	RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours = 0x00;
	RTC_TimeStructure.RTC_Minutes = 0x00;
	RTC_TimeStructure.RTC_Seconds = 0x00;

	/* Configure the RTC time register */
	if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
	{
		return -1;
	} 


	return 0;
}


void RtcInit(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;

	if (RtcInitialized) return;
	
	RtcInitialized = true;
	
	/* RTC configuration  */
	RTC_Config();

	/* Configure the RTC data register and RTC prescaler */
#ifdef RTC_CLOCK_SOURCE_LSI					//for one second tick
	RTC_InitStructure.RTC_AsynchPrediv = 19;
	RTC_InitStructure.RTC_SynchPrediv = 0x00;
#elif defined (RTC_CLOCK_SOURCE_LSE)
	RTC_InitStructure.RTC_AsynchPrediv = 0xff;
	RTC_InitStructure.RTC_SynchPrediv = 0x7f;
#endif
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;

	/* Check on RTC init */
	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
		return;
	}

	/* Configure the time register */
	RTC_Set_DateTime(); 


	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
  /* Enable the RTC Alarm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	return;
}

void rtc_irq_enable()
{
	irqChannelEnable(RTC_IRQn, true);
}

void rtc_irq_disable()
{
	irqChannelEnable(RTC_IRQn, false);
}
void print_timeout(int data1);
void RtcSetTimeout( uint32_t timeout )
{
		//print_timeout(timeout);
    RtcStartWakeUpAlarm( timeout );
}

TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
    if( timeout > McuWakeUpTime )
    {   // we have waken up from a GPIO and we have lost "McuWakeUpTime" that we need to compensate on next event
        if( NonScheduledWakeUp == true )
        {
            NonScheduledWakeUp = false;
            timeout -= McuWakeUpTime;
        }
    }

    if( timeout > McuWakeUpTime )
    {   // we don't go in Low Power mode for delay below 50ms (needed for LEDs)
        if( timeout < 50 ) // 50 ms
        {
            RtcTimerEventAllowsLowPower = false;
        }
        else
        {
            RtcTimerEventAllowsLowPower = true;
            timeout -= McuWakeUpTime;
        }
    }
    return  timeout;
}

TimerTime_t RtcGetTimerValue( void )
{
    return( RtcConvertCalendarTickToTimerTime( NULL ) );
}


TimerTime_t RtcGetElapsedAlarmTime( void )
{
    TimerTime_t currentTime = 0;
    TimerTime_t contextTime = 0;

    currentTime = RtcConvertCalendarTickToTimerTime( NULL );
    contextTime = RtcConvertCalendarTickToTimerTime( &RtcCalendarContext );

    if( currentTime < contextTime )
    {
        return( currentTime + ( 0xFFFFFFFF - contextTime ) );
    }
    else
    {
        return( currentTime - contextTime );
    }
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
    return( RtcGetTimerValue( ) + futureEventInTime );
}

TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if( eventInTime == 0 )
    {
        return 0;
    }

    elapsedTime = RtcConvertCalendarTickToTimerTime( NULL );

    if( elapsedTime < eventInTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - eventInTime ) );
    }
    else
    {
        return( elapsedTime - eventInTime );
    }
}

void BlockLowPowerDuringTask ( bool status )
{
    if( status == true )
    {
        RtcRecoverMcuStatus( );
    }
    LowPowerDisableDuringTask = status;
}

void RtcEnterLowPowerStopMode( void )
{
    if( ( LowPowerDisableDuringTask == false ) && ( RtcTimerEventAllowsLowPower == true ) )
    {
      //  BoardDeInitMcu( );

        // Disable the Power Voltage Detector
       // HAL_PWR_DisablePVD( );

      //  SET_BIT( PWR->CR, PWR_CR_CWUF );

        // Enable Ultra low power mode
        //HAL_PWREx_EnableUltraLowPower( );

        // Enable the fast wake up from Ultra low power mode
       // HAL_PWREx_EnableFastWakeUp( );

        // Enter Stop Mode
       PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    }
}

void RtcRecoverMcuStatus( void )
{
    // PWR_FLAG_WU indicates the Alarm has waken-up the MCU
  
    // check the clk source and set to full speed if we are coming from sleep mode
   // if( ( __HAL_RCC_GET_SYSCLK_SOURCE( ) == RCC_SYSCLKSOURCE_STATUS_HSI ) ||
   //     ( __HAL_RCC_GET_SYSCLK_SOURCE( ) == RCC_SYSCLKSOURCE_STATUS_MSI ) )
    {
     //   BoardInitMcu( );
    }
}
#define MIN(a,b)  (((a) < (b)) ? (a) : (b))

static uint32_t round(double dvalue)
{
	dvalue += 0.5;
	return (uint32_t)dvalue;
}
#if 0
static uint32_t ceil(double dvalue)
{
	dvalue += 0.99;
	return (uint32_t)dvalue;
}

static void RtcComputeWakeUpTime( void )
{
    uint32_t start = 0;
    uint32_t stop = 0;
    RTC_AlarmTypeDef  alarmRtc;
    RtcCalendar_t now;

    if( WakeUpTimeInitialized == false )
    {
        now = RtcGetCalendar( );
        RTC_GetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarmRtc);

        start = alarmRtc.RTC_AlarmTime.RTC_Seconds + ( SecondsInMinute * alarmRtc.RTC_AlarmTime.RTC_Minutes ) + ( SecondsInHour * alarmRtc.RTC_AlarmTime.RTC_Hours );
        stop = now.CalendarTime.RTC_Seconds + ( SecondsInMinute * now.CalendarTime.RTC_Minutes ) + ( SecondsInHour * now.CalendarTime.RTC_Hours );

        McuWakeUpTime = ceil ( ( stop - start ) * RTC_ALARM_TICK_DURATION );

        WakeUpTimeInitialized = true;
    }
}


static int RtcStart() {
	RTC_AlarmTypeDef  RTC_AlarmStructure;
	
	/* Disable the Alarm A */
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = 0;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 1;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 20;
	
	/* Set the Alarm A */
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0;    
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None; //RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

	/* Configure the RTC Alarm A register */
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

	//RTC_AlarmShow();

	/* Enable the RTC Alarm A Interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm  A */
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
	
	return 0;
}
#endif
static void RtcStartWakeUpAlarm( uint32_t timeoutValue )
{
	RtcCalendar_t now;
  RtcCalendar_t alarmTimer;
  RTC_AlarmTypeDef RTC_AlarmStructure;
	
	/* Disable the Alarm A */
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

  // Load the RTC calendar
  now = RtcGetCalendar( );

    // Save the calendar into RtcCalendarContext to be able to calculate the elapsed time
  RtcCalendarContext = now;

  // timeoutValue is in ms
  alarmTimer = RtcComputeTimerTimeToAlarmTick( timeoutValue, now );

	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = alarmTimer.CalendarTime.RTC_Hours;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = alarmTimer.CalendarTime.RTC_Minutes;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = alarmTimer.CalendarTime.RTC_Seconds;
	
	/* Set the Alarm A */
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = alarmTimer.CalendarDate.RTC_Date;    
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None; //RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

	/* Configure the RTC Alarm A register */
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

	/* Enable the RTC Alarm A Interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm  A */
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}

static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( TimerTime_t timeCounter, RtcCalendar_t now )
{
    RtcCalendar_t calendar = now;

    uint16_t seconds = now.CalendarTime.RTC_Seconds;
    uint16_t minutes = now.CalendarTime.RTC_Minutes;
    uint16_t hours = now.CalendarTime.RTC_Hours;
    uint16_t days = now.CalendarDate.RTC_Date;
    double timeoutValueTemp = 0.0;
    double timeoutValue = 0.0;
    double error = 0.0;

    timeCounter = MIN( timeCounter, ( TimerTime_t )( RTC_ALARM_MAX_NUMBER_OF_DAYS * SecondsInDay * RTC_ALARM_TICK_DURATION ) );

    if( timeCounter < 1 )
    {
        timeCounter = 1;
    }

    // timeoutValue is used for complete computation
    timeoutValue = round( timeCounter * RTC_ALARM_TICK_PER_MS );

    // timeoutValueTemp is used to compensate the cumulating errors in timing far in the future
    timeoutValueTemp =  ( double )timeCounter * RTC_ALARM_TICK_PER_MS;

    // Compute timeoutValue error
    error = timeoutValue - timeoutValueTemp;

    // Add new error value to the cumulated value in uS
    TimeoutValueError += ( error  * 1000 );

    // Correct cumulated error if greater than ( RTC_ALARM_TICK_DURATION * 1000 )
    if( TimeoutValueError >= ( int32_t )( RTC_ALARM_TICK_DURATION * 1000 ) )
    {
        TimeoutValueError = TimeoutValueError - ( uint32_t )( RTC_ALARM_TICK_DURATION * 1000 );
        timeoutValue = timeoutValue + 1;
    }

    // Convert milliseconds to RTC format and add to now
    while( timeoutValue >= SecondsInDay )
    {
        timeoutValue -= SecondsInDay;
        days++;
    }

    // Calculate hours
    while( timeoutValue >= SecondsInHour )
    {
        timeoutValue -= SecondsInHour;
        hours++;
    }

    // Calculate minutes
    while( timeoutValue >= SecondsInMinute )
    {
        timeoutValue -= SecondsInMinute;
        minutes++;
    }

    // Calculate seconds
    seconds += timeoutValue;

    // Correct for modulo
    while( seconds >= 60 )
    {
        seconds -= 60;
        minutes++;
    }

    while( minutes >= 60 )
    {
        minutes -= 60;
        hours++;
    }

    while( hours >= HoursInDay )
    {
        hours -= HoursInDay;
        days++;
    }

    if( ( now.CalendarDate.RTC_Year == 0 ) || ( ( now.CalendarDate.RTC_Year + Century ) % 4 ) == 0 )
    {
        if( days > DaysInMonthLeapYear[now.CalendarDate.RTC_Month - 1] )
        {
            days = days % DaysInMonthLeapYear[now.CalendarDate.RTC_Month - 1];
        }
    }
    else
    {
        if( days > DaysInMonth[now.CalendarDate.RTC_Month - 1] )
        {
            days = days % DaysInMonth[now.CalendarDate.RTC_Month - 1];
        }
    }

    calendar.CalendarTime.RTC_Seconds = seconds;
    calendar.CalendarTime.RTC_Minutes = minutes;
    calendar.CalendarTime.RTC_Hours = hours;
    calendar.CalendarDate.RTC_Date = days;

    return calendar;
}

//
// REMARK: Removed function static attribute in order to suppress
//         "#177-D function was declared but never referenced" warning.
// static RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
//
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
{
    RtcCalendar_t calendar = { 0 };

    uint16_t seconds = 0;
    uint16_t minutes = 0;
    uint16_t hours = 0;
    uint16_t days = 0;
    uint8_t months = 1; // Start at 1, month 0 does not exist
    uint16_t years = 0;
    uint16_t century = 0;
    double timeCounterTemp = 0.0;

    timeCounterTemp = ( double )timeCounter * RTC_ALARM_TICK_PER_MS;

    // Convert milliseconds to RTC format and add to now
    while( timeCounterTemp >= SecondsInLeapYear )
    {
        if( ( years == 0 ) || ( years % 4 ) == 0 )
        {
            timeCounterTemp -= SecondsInLeapYear;
        }
        else
        {
            timeCounterTemp -= SecondsInYear;
        }
        years++;
        if( years == 100 )
        {
            century = century + 100;
            years = 0;
        }
    }

    if( timeCounterTemp >= SecondsInYear )
    {
        if( ( years == 0 ) || ( years % 4 ) == 0 )
        {
            // Nothing to be done
        }
        else
        {
            timeCounterTemp -= SecondsInYear;
            years++;
        }
    }

    if( ( years == 0 ) || ( years % 4 ) == 0 )
    {
        while( timeCounterTemp >= ( DaysInMonthLeapYear[ months - 1 ] * SecondsInDay ) )
        {
            timeCounterTemp -= DaysInMonthLeapYear[ months - 1 ] * SecondsInDay;
            months++;
        }
    }
    else
    {
        while( timeCounterTemp >= ( DaysInMonth[ months - 1 ] * SecondsInDay ) )
        {
            timeCounterTemp -= DaysInMonth[ months - 1 ] * SecondsInDay;
            months++;
        }
    }

    // Convert milliseconds to RTC format and add to now
    while( timeCounterTemp >= SecondsInDay )
    {
        timeCounterTemp -= SecondsInDay;
        days++;
    }

    // Calculate hours
    while( timeCounterTemp >= SecondsInHour )
    {
        timeCounterTemp -= SecondsInHour;
        hours++;
    }

    // Calculate minutes
    while( timeCounterTemp >= SecondsInMinute )
    {
        timeCounterTemp -= SecondsInMinute;
        minutes++;
    }

    // Calculate seconds
    seconds = round( timeCounterTemp );

    calendar.CalendarTime.RTC_Seconds = seconds;
    calendar.CalendarTime.RTC_Minutes = minutes;
    calendar.CalendarTime.RTC_Hours = hours;
    calendar.CalendarDate.RTC_Date = days;
    calendar.CalendarDate.RTC_Month = months;
    calendar.CalendarDate.RTC_Year = years;
    calendar.CalendarCentury = century;

    return calendar;
}

static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar )
{
    TimerTime_t timeCounter = 0;
    RtcCalendar_t now;
    double timeCounterTemp = 0.0;
		uint16_t i;

    // Passing a NULL pointer will compute from "now" else,
    // compute from the given calendar value
    if( calendar == NULL )
    {
        now = RtcGetCalendar( );
    }
    else
    {
        now = *calendar;
    }

    // Years (calculation valid up to year 2099)
    for(i = 0; i < ( now.CalendarDate.RTC_Year + now.CalendarCentury ); i++ )
    {
        if( ( i == 0 ) || ( i % 4 ) == 0 )
        {
            timeCounterTemp += ( double )SecondsInLeapYear;
        }
        else
        {
            timeCounterTemp += ( double )SecondsInYear;
        }
    }

    // Months (calculation valid up to year 2099)*/
    if( ( now.CalendarDate.RTC_Year == 0 ) || ( ( now.CalendarDate.RTC_Year + now.CalendarCentury ) % 4 ) == 0 )
    {
        for( i = 0; i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( double )( DaysInMonthLeapYear[i] * SecondsInDay );
        }
    }
    else
    {
        for( i = 0;  i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( double )( DaysInMonth[i] * SecondsInDay );
        }
    }

    timeCounterTemp += ( double )( ( uint32_t )now.CalendarTime.RTC_Seconds +
                     ( ( uint32_t )now.CalendarTime.RTC_Minutes * SecondsInMinute ) +
                     ( ( uint32_t )now.CalendarTime.RTC_Hours * SecondsInHour ) +
                     ( ( uint32_t )( now.CalendarDate.RTC_Date * SecondsInDay ) ) );

    timeCounterTemp = ( double )timeCounterTemp * RTC_ALARM_TICK_DURATION;

    timeCounter = round( timeCounterTemp );
    return ( timeCounter );
}

static void RtcCheckCalendarRollOver( uint8_t year )
{
    if( year == 99 )
    {
        CalendarRollOverReady = true;
    }

    if( ( CalendarRollOverReady == true ) && ( ( year + Century ) == Century ) )
    {   // Indicate a roll-over of the calendar
        CalendarRollOverReady = false;
        Century = Century + 100;
    }
}

static RtcCalendar_t RtcGetCalendar( void )
{
    RtcCalendar_t calendar;
    RTC_GetTime( RTC_Format_BIN, &calendar.CalendarTime);
    RTC_GetDate( RTC_Format_BIN, &calendar.CalendarDate);
    calendar.CalendarCentury = Century;
    RtcCheckCalendarRollOver( calendar.CalendarDate.RTC_Year );
    return calendar;
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
 void RtcSetTimeout( uint32_t timeout );
void TimerIrqHandler( void );
#include "contiki.h"

void RTC_Alarm_IRQHandler( void )
{
	 //for rtc alarm timer
   //TimerIrqHandler( );

	 //for contiki timer
	 if(etimer_pending() && etimer_next_expiration_time() <= clock_time()) {
		  etimer_request_poll();
	 }
}

/*
for contiki timer
*/
clock_time_t clock_time(void)
{
 	return RtcGetTimerValue();
}

unsigned long clock_seconds(void)
{
	return RtcGetTimerValue()/CLOCK_CONF_SECOND;
}

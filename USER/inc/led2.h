#ifndef __LED2_H__
#define __LED2_H__

#define BLINK_TIME  (CLOCK_SECOND/2)

enum LED_STATUS {
	LED_STATUS_BLINK_TWO,
	LED_STATUS_BLINK_RED,
	LED_STATUS_BLINK_BLUE,
	LED_STATUS_ALWAYS_ON_RED,
	LED_STATUS_ALWAYS_ON_BLUE,
	LED_STATUS_ALWAYS_ON_ALL,
	LED_STATUS_ALWAYS_OFF
};
void led_status_set(enum LED_STATUS ledstatus);
void refresh_led_status(void);
#endif

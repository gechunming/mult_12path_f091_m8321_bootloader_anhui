#ifndef __LED__H
#define __LED__H
void led_init(void);
void led_set(uint8_t index, uint8_t val);
void led_blink(uint8_t index, uint8_t blinktimes, uint16_t ontime, uint16_t offtime);
#endif


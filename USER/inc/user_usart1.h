#ifndef _USER_USART1_H_
#define _USER_USART1_H_

#include "stm32f0xx.h"

void print_string(char *str);
void print_rssi(char rssi);
void print_int(uint8_t *str, int val);
#endif

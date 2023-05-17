#ifndef __RTC_H__
#define __RTC_H__

#include "gd32f10x.h"
#include <stdio.h>

void nvic_configuration(void);
void rtc_configuration(void);
uint32_t time_regulate(void);
void time_adjust(void);
void time_display(uint32_t timevar);
void time_show(void);
uint8_t usart_scanf(uint32_t value);

#endif
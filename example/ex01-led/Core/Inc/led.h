#ifndef __LED_H__
#define __LED_H__

#include "main.h"
#include "gpio.h"
#include "tim.h"

bool ledOnOff(bool is_led_on);
bool ledEasingOnOff(bool is_led_on, uint32_t easing_time);

#endif
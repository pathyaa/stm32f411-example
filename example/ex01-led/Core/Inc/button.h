#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "main.h"
#include "gpio.h"

enum
{
	IDLE,
	PRESSED_CHECK,
	PRESSED,
	RELEASED_CHECK,
	RELEASED
};

bool isButtonClicked(void);

#endif
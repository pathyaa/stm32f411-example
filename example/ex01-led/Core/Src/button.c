#include "button.h"

bool isButtonClicked(void)
{
  static uint8_t step = 0;
  static uint32_t tick = 0;
  static bool isPressed = false;
  bool ret = false;

  switch (step)
  {
    case IDLE:
      if (gpioRead(BTN) == HIGH)
      {
        step = RELEASED_CHECK;
        tick = HAL_GetTick();
      }
      else if (gpioRead(BTN) == LOW)
      {
        tick = HAL_GetTick();
        step = PRESSED_CHECK;
      }
    break;
    case PRESSED_CHECK:
    if (HAL_GetTick() - tick >= 20)
    {
      if (gpioRead(BTN) == LOW)
      {
        isPressed = true;
        step = PRESSED;
      }
      else
      {
        step = RELEASED_CHECK;
        tick = HAL_GetTick();
      }
    }
    break;
    case PRESSED:
    if (gpioRead(BTN) == HIGH)
    {
      tick = HAL_GetTick();
      step = RELEASED_CHECK;
    }
    break;
    case RELEASED_CHECK:
    if (HAL_GetTick() - tick >= 20)
    {
      if (!isPressed)
      {
        if (gpioRead(BTN) == HIGH)
        {
          step = IDLE;
        }
        else if (gpioRead(BTN) == LOW)
        {
          tick = HAL_GetTick();
          step = PRESSED_CHECK;
        }
      }
      else 
      {
        isPressed = false;
        if (gpioRead(BTN) == HIGH)
        {
          step = RELEASED;
        }
        else
        {
          tick = HAL_GetTick();
          step = PRESSED_CHECK;
        }
      }
    }
    break;
    case RELEASED:
    step = IDLE;
    ret = true;
    break;
  }
  return ret;
}
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
bool gpioToggle(uint8_t pin)
{
  bool ret = false;

  for (uint8_t i=LED; i<GPIO_MAX; i++)
  {
    if (i==pin)
    {
      if (gpioRead(pin) == HIGH)
      {
        gpioWrite(pin, LOW);
        ret = true;
      }
      else if (gpioRead(pin) == LOW)
      {
        gpioWrite(pin, HIGH);
        ret = true;
      }
    }
  }
  
  return ret;
}

bool gpioWrite(uint8_t pin, uint8_t pin_state)
{
  switch (pin)
  {
    
    case LED:
      if (pin_state == HIGH)
      {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        return true;
      }
      else if (pin_state == LOW)
      {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        return true;
      }
    break;
    case BTN:
      if (pin_state == HIGH)
      {
        HAL_GPIO_WritePin(BTN_GPIO_Port, BTN_Pin, GPIO_PIN_SET);
        return true;
      }
      else if (pin_state == LOW)
      {
        HAL_GPIO_WritePin(BTN_GPIO_Port, BTN_Pin, GPIO_PIN_RESET);
        return true;
      }
    break;
  }
  return false;
}

bool gpioRead(uint8_t pin)
{
  bool ret = false;

  switch (pin)
  {
    case LED:
    if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET)
    {
      ret = HIGH;
    }
    else if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_RESET)
    {
      ret = LOW;
    }
    break;
    case BTN:
    if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET)
    {
      ret = HIGH;
    }
    else if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET)
    {
      ret = LOW;
    }
    break;
  }
  return ret;
}
/* USER CODE END 2 */

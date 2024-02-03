#include "led.h"

bool ledOnOff(bool is_led_on)
{
    if (is_led_on)
    {
        gpioWrite(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    else
    {
        gpioWrite(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
}

bool ledEasingOnOff(bool is_led_on, uint8_t duty)
{
    
}
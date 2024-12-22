//
// Created by DUSK on 2024/12/22.
//
#include "stm32f4xx_hal.h"
#include "led.h"


#define LED_PIN GPIO_PIN_4
#define LED_PORT GPIOF

void led_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();

//    引脚是PF4
    GPIO_InitStructure.Pin = LED_PIN|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStructure);
}


void led_start()
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
}

void led_stop()
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
}

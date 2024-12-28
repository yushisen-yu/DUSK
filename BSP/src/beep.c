//
// Created by DUSK on 2024/12/22.
//
#include "stm32f4xx_hal.h"
#include "beep.h"


#define BEEP_PIN GPIO_PIN_13
#define BEEP_PORT GPIOC
void beep_init()
{
    // 引脚是PC13
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = BEEP_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);


}

void beep_start()
{
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET);
}

void beep_stop()
{
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);
}
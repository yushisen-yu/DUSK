//
// Created by DUSK on 2024/12/28.
//

#include "stm32f4xx_hal.h"
#include "DCMotor.h"


#define DCMoter_PIN GPIO_PIN_8
#define DCMoter_PORT GPIOB

void DCMotor_init()
{
    GPIO_InitTypeDef GPIO_InitStructure= {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
//DCMOTOR_A正转     PC13

    GPIO_InitStructure.Pin = DCMoter_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DCMoter_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(DCMoter_PORT,DCMoter_PIN, GPIO_PIN_RESET);
}

void DCMotor_start()
{
  HAL_GPIO_WritePin(DCMoter_PORT,DCMoter_PIN, GPIO_PIN_SET);
}

void DCMotor_stop()
{
  HAL_GPIO_WritePin(DCMoter_PORT,DCMoter_PIN, GPIO_PIN_RESET);
}

//
// Created by DUSK on 2024/12/28.
//

#include "stm32f4xx_hal.h"
#include "DCMotor.h"
#include "PWM.h"

#define DCMOTOR_A_PIN GPIO_PIN_8
#define DCMOTOR_B_PIN GPIO_PIN_9
#define DCMOTOR_PORT GPIOB

//MOTOR_A   PB8 反转
#define    DCMOTOR_A_L         HAL_GPIO_WritePin(DCMOTOR_PORT, DCMOTOR_A_PIN, GPIO_PIN_RESET)
#define    DCMOTOR_A_H         HAL_GPIO_WritePin(DCMOTOR_PORT, DCMOTOR_A_PIN, GPIO_PIN_SET)

//MOTOR_B   PB9 正转
#define    DCMOTOR_B_L         HAL_GPIO_WritePin(DCMOTOR_PORT, DCMOTOR_B_PIN, GPIO_PIN_RESET)
#define    DCMOTOR_B_H         HAL_GPIO_WritePin(DCMOTOR_PORT, DCMOTOR_B_PIN, GPIO_PIN_SET)

void DCMotor_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // DCMOTOR_A正转

    GPIO_InitStructure.Pin =/*DCMOTOR_A_PIN|*/DCMOTOR_B_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DCMOTOR_PORT, &GPIO_InitStructure);

    // 默认停止
    /* DCMOTOR_A_L;*/
    DCMOTOR_B_L;

    TIM10_PWM_Init(1000 - 1, 84 - 1);
    TIM10_PWM_SetCompare(0);


}

// 正转
void DCMotor_forward(uint32_t speed)
{
    DCMOTOR_A_L;
    DCMOTOR_B_H;
    TIM10_PWM_SetCompare(speed);
}

// 反转无效
void DCMotor_reverse(uint32_t speed)
{
    DCMOTOR_A_H;
    DCMOTOR_B_L;
    TIM10_PWM_SetCompare(speed);
}

// 停止
void DCMotor_stop()
{
    DCMOTOR_A_L;
    DCMOTOR_B_L;
    TIM10_PWM_SetCompare(0);
}
#include "PWM.h"

TIM_HandleTypeDef htim10;

/**********************************************************************************************************
函数名称：TIM10_PWM_Init初始化函数
输入参数：自动重装值、时钟预分频数
输出参数：无
函数返回：无
TIM10_CH1    PB8
**********************************************************************************************************/
void TIM10_PWM_Init(uint32_t arr, uint32_t psc)
{
    TIM_OC_InitTypeDef sConfigOC;

    // 使能定时器和GPIO的时钟
    __HAL_RCC_TIM10_CLK_ENABLE();       // TIM10时钟使能
    __HAL_RCC_GPIOB_CLK_ENABLE();       // GPIOB时钟使能

    // 配置GPIO引脚作为复用推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;   // PB8
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10; // 设置为TIM10的复用功能
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 初始化TIM10
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = psc;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = arr;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }

    // 配置PWM通道
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // 初始占空比为0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        // Configuration Error
        Error_Handler();
    }
    TIM10_PWM_Start();
    TIM10_PWM_SetCompare(1000);

}


void TIM10_PWM_SetCompare(uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, compare);
}

void TIM10_PWM_Start()
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void TIM10_PWM_Stop()
{
    HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
}
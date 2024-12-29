#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//  函数声明
void TIM10_PWM_Init(uint32_t arr, uint32_t psc);

void TIM10_PWM_SetCompare(uint32_t compare);

void TIM10_PWM_Start();

void TIM10_PWM_Stop();



#ifdef __cplusplus
}
#endif

#endif	

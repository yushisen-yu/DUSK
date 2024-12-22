//
// Created by DUSK on 2024/12/22.
//

#ifndef FURINA_DELAY_H
#define FURINA_DELAY_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif



void delay_init();

// 延时微秒函数
void delay_us(uint16_t us);


#ifdef __cplusplus
}
#endif
#endif //FURINA_DELAY_H

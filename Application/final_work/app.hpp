//
// Created by DUSK on 2024/12/17.
//

#ifndef FURINA_APP_H
#define FURINA_APP_H
// 宏定义
#define APP_NO_RTOS // 没有使用FreeRTOS

/*************************模块级**************************/
#define USE_WAVE_SIGNAL // 使用波形信号

/**************************板级**************************/
#define USE_TIMER // 使用定时器
#define USE_ADC  // 使用ADC
#define USE_DAC // 使用DAC
#endif //FURINA_APP_H
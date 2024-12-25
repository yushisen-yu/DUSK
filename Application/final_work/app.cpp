//
// Created by DUSK on 2024/12/17.
//

#include "app.hpp"
#include "timer.h"
#include "dac.h"
#include "wave_signal.hpp"
#include "key.hpp"
#include "GUI.hpp"
#include "WaveCurve.hpp"
#include "gt9147.h"
#include "delay.h"
#include "beep.h"
#include "led.h"
#include "DHT11.h"



void app_init()
{
  beep_init();
  led_init();
//  DHT11_Init();
}

int16_t x, y;

// 后台运算
void background_handler() {


//    GT9147_Read(&x, &y);
}
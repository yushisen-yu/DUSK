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

void app_init()
{

}



int16_t x, y;

// 后台运算
void background_handler()
{


    GT9147_Read(&x,&y);
}
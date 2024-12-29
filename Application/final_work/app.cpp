//
// Created by DUSK on 2024/12/17.
//

#include "app.hpp"
#include "timer.h"
#include "dac.h"
#include "wave_signal.hpp"
#include "key.hpp"

#ifdef GUI_ENABLE
#include "GUI.hpp"
#include "WaveCurve.hpp"
#endif

#include "gt9147.h"
#include "delay.h"
#include "beep.h"
#include "led.h"
#include "DHT11.h"
#include "DCMotor.h"


constexpr uint32_t TEMP_HUMI_CHECK_DELAY = 2000;// 2s检测一次

class DHT
{
public:
    static void init();//用于初始内部延时计时器
    static void measure();// 测量
    static const float& get_temp(){ return temp;}// 获取温度
    static const float& get_humi(){ return humi;}// 获取湿度


private:
    static inline uint32_t start_tick = 0;
    static inline uint32_t final_tick = 0;
    static inline uint8_t DHT_flag = 0;// 开始测量温湿度
    static inline float temp = 0;
    static inline float humi = 0;
};

void app_init()
{
    beep_init();
    led_init();
    DCMotor_init();
    // 初始化温湿度传感器
    DHT11_Init();
    DHT::init();

}
// 后台运算
void background_handler()
{
    // 检测温湿度
    DHT::measure();


}

// ----------------------类的实现接口-----------------------------

void DHT::init()
{
    start_tick = HAL_GetTick();// 获取起点
    final_tick = start_tick + TEMP_HUMI_CHECK_DELAY;// 计算终点
}

/**
 * @brief 测量温度和湿度
 * @note 非阻塞式测量温湿度，测量间隔为2s
 */
void DHT::measure()
{
    // 非阻塞延迟
    if (final_tick < start_tick)
    {
        // 检测溢出情况
        if (HAL_GetTick() > final_tick && HAL_GetTick() < start_tick)
        {
            // 重置
            start_tick = HAL_GetTick();
            final_tick = start_tick + TEMP_HUMI_CHECK_DELAY;
            // 标志记为1
            DHT_flag = 1;
        }
    }
    else
    {
        if (HAL_GetTick() > final_tick)
        {
            // 重置
            start_tick = HAL_GetTick();
            final_tick = start_tick + TEMP_HUMI_CHECK_DELAY;
            // 标志记为1
            DHT_flag = 1;
        }
    }

    // 测量温湿度
    if (DHT_flag)
    {
        DHT_flag = 0;
        if (!DHT11_Read_Data_Fast_Pro(&temp, &humi))
        {
            // 处理检测失败的情况
        }
    }
}


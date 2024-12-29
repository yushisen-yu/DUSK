//
// Created by DUSK on 2024/12/17.
//

#include "app.hpp"
#include "timer.h"
#include "dac.h"
#include "wave_signal.hpp"
#include "key.hpp"
#include "adxl345.h"


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
constexpr uint32_t ACC_CHECK_DELAY = 200;

enum class DHT_FLAGS : uint8_t
{
    START = 1 << 0,//bit:0 是否开启测量
    DELAY = 1 << 1,//bit:1 检测间隔是否达到
    TYPE = 1 << 2,//bit:2 传感器类型,0表示DHT，1表示ACC
};

class DHT_ACC
{
public:
    static void init();//用于初始内部延时计时器
    static void measure();// 测量
    static const float &get_temp() { return temp; }// 获取温度
    static const float &get_humi() { return humi; }// 获取湿度

    // 0表示DHT11，1表示ACC
    static void switch_sensor(bool type);


public:
    // 辅助函数：设置指定标志
    static void set_flag(DHT_FLAGS flag)
    {
        flags |= static_cast<uint8_t>(flag);
    }

    // 辅助函数：清除指定标志
    static void clear_flag(DHT_FLAGS flag)
    {
        flags &= ~static_cast<uint8_t>(flag);
    }

    // 辅助函数：获取指定标志
    static uint8_t get_flag(DHT_FLAGS flag)
    {
        return flags & static_cast<uint8_t>(flag);
    }

    // 辅助函数：翻转指定标志
    static void toggle_flag(DHT_FLAGS flag)
    {
        flags ^= static_cast<uint8_t>(flag);
    }

private:
    static inline uint32_t start_tick = 0;
    static inline uint32_t final_tick = 0;
    static inline uint32_t delay_tick = TEMP_HUMI_CHECK_DELAY;
    static inline uint8_t flags = 0;// 开始测量温湿度
    static inline float temp = 0;
    static inline float humi = 0;
    static inline short x = 0;
    static inline short y = 0;
    static inline short z = 0;
};



//

void app_init()
{
    beep_init();
    led_init();
    // 初始化直流电机
    DCMotor_init();
    // 初始化温湿度传感器
    DHT11_Init();
    DHT_ACC::init();
    // 初始化重力加速度传感器
    ADXL345_Init();
    HAL_Delay(100);


}

// 后台运算


void background_handler()
{
    // 检测温湿度
    DHT_ACC::measure();

}

// ----------------------类的实现接口-----------------------------

/**
 * @brief 初始化内部延时计时器
 * @note 不进行初始化，可能会导致延迟计数器紊乱
 */
void DHT_ACC::init()
{
    start_tick = HAL_GetTick();// 获取起点
    final_tick = start_tick + delay_tick;// 计算终点
}

/**
 * @brief 测量温度和湿度
 * @note 非阻塞式测量温湿度，测量间隔为2s。已经添加了UI显示
 */
short angle1, angle2;

void DHT_ACC::measure()
{

    if (get_flag(DHT_FLAGS::START))
    {
        // 非阻塞延迟
        if (final_tick < start_tick)
        {
            // 检测溢出情况
            if (HAL_GetTick() > final_tick && HAL_GetTick() < start_tick)
            {
                // 重置
                start_tick = HAL_GetTick();
                final_tick = start_tick + delay_tick;
                // 标志记为1
                set_flag(DHT_FLAGS::DELAY);
            }
        }
        else
        {
            if (HAL_GetTick() > final_tick)
            {
                // 重置
                start_tick = HAL_GetTick();
                final_tick = start_tick + delay_tick;
                // 标志记为1
                set_flag(DHT_FLAGS::DELAY);
            }
        }

        // 测量温湿度
        if (get_flag(DHT_FLAGS::DELAY))
        {
            clear_flag(DHT_FLAGS::DELAY);

            // 检测传感器类型
            if (get_flag(DHT_FLAGS::TYPE))
            {
                // ACC
                ADXL345ReadAvval_Once(&x, &y, &z);

                angle1 = ADXL345Get_Angle(x, y, z, 1);
                angle2 = ADXL345Get_Angle(x, y, z, 2);
#ifdef GUI_ENABLE
                UI::add_temp_data(angle1);
                UI::add_humi_data(angle2);
#endif

            }
            else
            {
                // DHT11
                if (DHT11_Read_Data_Fast_Pro(&temp, &humi))
                {
#ifdef GUI_ENABLE
                    UI::add_temp_data((short) temp);
                    UI::add_humi_data((short) humi);
#endif
                }
            }

        }
    }
}

void DHT_ACC::switch_sensor(bool type)
{
    if (type)
    {
        set_flag(DHT_FLAGS::TYPE);
        delay_tick = ACC_CHECK_DELAY;
    }
    else
    {
        clear_flag(DHT_FLAGS::TYPE);
        delay_tick = TEMP_HUMI_CHECK_DELAY;
    }
}

void start()
{
    DHT_ACC::set_flag(DHT_FLAGS::START);
    DHT_ACC::init();
}

void stop()
{
    DHT_ACC::clear_flag(DHT_FLAGS::START);
}

void switch_sensor(bool type)
{
    DHT_ACC::switch_sensor(type);
}


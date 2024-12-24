//
// Created by DUSK on 2024/12/24.
//

#include "DHT11.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "stdbool.h"


#define DHT11_Pin GPIO_PIN_6
#define DHT11_Pin_Location 6
#define DHT11_GPIO_Port GPIOE
#define DHT11_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define DHT11_MAX_DELAY_COUNT 4000//防止卡死
#define USE_YZHX 1                //优化等级，分为0,1,2,3

#define DHT11_Read() (DHT11_GPIO_Port->IDR & DHT11_Pin) /*HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)*/

#define DHT11_High() DHT11_GPIO_Port->ODR |= (0x01 << DHT11_Pin_Location)
#define DHT11_Low() DHT11_GPIO_Port->ODR &= ~(0x01 << DHT11_Pin_Location) /*HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET)*/

#define DHT11_Wait_Low() while (DHT11_Read())
#define DHT11_Wait_High() while (!DHT11_Read())

#define DHT11_IN()                                                  \
    {                                                               \
        DHT11_GPIO_Port->MODER &= ~(3 << (2 * DHT11_Pin_Location)); \
        DHT11_GPIO_Port->MODER |= 0 << 2 * DHT11_Pin_Location;      \
    }
#define DHT11_OUT()                                                 \
    {                                                               \
        DHT11_GPIO_Port->MODER &= ~(3 << (2 * DHT11_Pin_Location)); \
        DHT11_GPIO_Port->MODER |= 1 << 2 * DHT11_Pin_Location;      \
    }

//static uint16_t std_delay_80us = 875;//事先测试过
//static uint16_t std_delay_50us = 566;

/**动态计算延时，以确保任何情况下都可以得到较为准确的延时*/
//void std_delay_us(uint8_t us)
//{
//    //   uint16_t count = std_delay_80us * us / 80;//测试得到的
//    uint16_t count = 11 * us;
//    for (uint16_t i = 0; i < count; ++i)
//        ;
//}
inline void std_delay_25us()
{
  for (uint16_t i = 0; i < 20; ++i)//单个任务时，大概为273
    ;
}


void DHT11_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  DHT11_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  //    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;//输入模式下，最好不要配置速度，所以为了兼容输入就不配置了，即默认2MHz
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
  DHT11_High();
}


inline void DHT11_Rst()
{
  DHT11_OUT();
  DHT11_Low();
  delay_us(30)//根据时序图可知，需要至少拉低18ms
  DHT11_High();
  std_delay_25us();//20-40us
}

inline void DHT11_Check()
{
  DHT11_IN();
  //等待低电平
  DHT11_Wait_Low();
  //等待高电平
  DHT11_Wait_High();
  // 等待低电平
  DHT11_Wait_Low();
}

#if USE_YZHX == 0
bool DHT11_Read_Data_Fast_Pro(float &temp, float &humi)
{
    static uint8_t buf[5];

    DHT11_Rst();  // 设置输出模式
    DHT11_Check();// 设置输入模式

    for (unsigned char &i: buf)// 读取40位数据
    {
        uint8_t data = 0;
        for (uint8_t j = 0; j < 8; j++)
        {
            data <<= 1;
            while (DHT11_Read()) {} // 等待低电平
            while (!DHT11_Read()) {}// 等待变高电平

            // 开始读数据
            uint16_t time_count;
            for (time_count = 0; DHT11_Read(); ++time_count) {}
            data |= time_count >> 10;// 由于事先已经知道一个为1194，一个为406左右
        }
        i = data;// 存储数据
    }

    if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
    {
        humi = (buf[0] * 10 + buf[1]) / 10.0f;
        temp = (buf[2] * 10 + buf[3]) / 10.0f;
        return true;
    }
    else
    {
        return false;
    }
}
#endif

/********************下面为次优级优化********************/
#if USE_YZHX == 1
// 全局变量
static uint8_t timeBuf[40];// 存储计数值
static uint8_t timeBufIndex = 0;

//void DHT11_Read_Byte_Fast_Pro()
//{
//    for (uint8_t i = 0; i < 8; i++)
//    {
//        while (DHT11_Read()) {} // 等待低电平
//        while (!DHT11_Read()) {}// 等待变高电平
//
//        // 开始读数据
//        uint16_t time_count;
//        for (time_count = 0; DHT11_Read(); ++time_count) {}
//        timeBuf[timeBufIndex++] = time_count>>4;// 存储计数值,由于事先已经知道一个为875，一个为275左右，所以除以16
//    }
//}

bool DHT11_Read_Data_Fast_Pro( float &temp, float &humi)
{
static uint8_t buf[5];

DHT11_Rst();  // 设置输出模式
DHT11_Check();// 设置输入模式

timeBufIndex = 0;          // 重置计数值索引
for (unsigned char &i: buf)// 读取40位数据
{
//        DHT11_Read_Byte_Fast_Pro();
//读取一字节
for (uint8_t j = 0; j < 8; j++)
{
DHT11_Wait_High();// 等待变高电平

// 开始读数据
uint16_t time_count = 0;
for (; DHT11_Read() && time_count < DHT11_MAX_DELAY_COUNT; ++time_count) {}
if (time_count >= DHT11_MAX_DELAY_COUNT)
{
return false;
}

timeBuf[timeBufIndex++] = time_count >> 4;// 存储计数值,由于事先已经知道一个为875，一个为275左右，所以除以16
}
}

//    std_delay_25us();
//    std_delay_25us();
//    DHT11_OUT();
//    DHT11_High();

uint16_t timeMax = 0;
uint16_t timeMin = 0xFFFF;
for (unsigned short i: timeBuf)
{
if (i > timeMax) timeMax = i;
if (i < timeMin) timeMin = i;
}

uint16_t timeMed = (timeMax + timeMin) >> 1;// 取中位数
for (uint8_t i = 0; i < 5; ++i)
{
uint8_t data = 0;
for (uint8_t j = 0; j < 8; j++)
{
data <<= 1;
data |= (timeBuf[i * 8 + j] > timeMed);
}
buf[i] = data;// 存储数据
}

if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
{
humi = (buf[0] * 10 + buf[1]) / 10.0f;
temp = (buf[2] * 10 + buf[3]) / 10.0f;
return true;
}
else
{
return false;
}
}
#endif

#if USE_YZHX == 2
static uint16_t timeBuf[40];//存储计数值
static uint8_t timeBufIndex = 0;
void DHT11_Read_Byte_Fast_Pro()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        while (DHT11_Read())
            ;//等待低电平
        //变低了说明上一次数据位读取结束

        while (!DHT11_Read())
            ;//等待变高电平
        //变高了说明数据位读取开始

        /**开始读数据*/
        //低电平：26-28us   高电平：70us
        uint16_t time_count;
        for (time_count = 0; DHT11_Read(); ++time_count)
            ;                                //等待低电平
        timeBuf[timeBufIndex++] = time_count;//存储计数值//存储计数值
    }
}


bool DHT11_Read_Data_Fast_Pro(float &temp, float &humi)
{
    static uint8_t buf[5];

    DHT11_Rst();               //在里面设置了输出模式
    DHT11_Check();             //在里面设置了输入模式
                               //  return false;//如果超时，则退出
    timeBufIndex = 0;          //存储计数值索引
    for (unsigned char &i: buf)//读取40位数据
    {
        DHT11_Read_Byte_Fast_Pro();
    }

    uint16_t timeMax = 0;
    uint16_t timeMin = 0xFFFF;
    for (unsigned short i: timeBuf)
    {
        if (i > timeMax)
        {
            timeMax = i;
        }
        else if (i < timeMin)
        {
            timeMin = i;
        }
    }

    /**把计数值转为二进制数据*/
    uint8_t data;                               //临时数据
    uint16_t timeMed = (timeMax + timeMin) >> 1;//整除2，取中位数
    bool tempBin;
    for (uint8_t i = 0; i < 5; ++i)
    {
        data = 0;
        for (uint8_t j = 0; j < 8; j++)
        {
            data <<= 1;
            //比较计数值，读取二进制数据
            if (timeBuf[i * 8 + j] > timeMed)
            {
                tempBin = true;
            }
            else
            {
                tempBin = false;
            }
            data |= tempBin;
        }
        buf[i] = data;//存储数据
    }

    /**检验**/
    if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
    {
        humi = (float) (buf[0] + buf[1] * 0.1);
        temp = (float) (buf[2] + buf[3] * 0.1);
        return true;
    }
    else
    {
        return false;
    }
}
#endif


/********************下面为原版优化********************/
#if USE_YZHX == 3
static uint16_t timeBuf[40];//存储计数值

void DHT11_Read_Byte_Fast_Pro()
{
    static uint8_t timeBufIndex = 0;//存储计数值索引
    for (uint8_t i = 0; i < 8; i++)
    {
        while (DHT11_Read())
            ;//等待低电平
        //变低了说明上一次数据位读取结束

        while (!DHT11_Read())
            ;//等待变高电平
        //变高了说明数据位读取开始

        /**开始读数据*/
        //低电平：26-28us   高电平：70us
        uint16_t time_count;
        for (time_count = 0; DHT11_Read(); ++time_count)
            ;                                //等待低电平
        timeBuf[timeBufIndex++] = time_count;//存储计数值
    }
}


bool DHT11_Read_Data_Fast_Pro(float &temp, float &humi)
{
    static uint8_t buf[5];
    static uint16_t timeMax = 0;
    static uint16_t timeMin = 0xFFFF;

    DHT11_Rst();               //在里面设置了输出模式
    DHT11_Check();             //在里面设置了输入模式
                               //  return false;//如果超时，则退出
    for (unsigned char &i: buf)//读取40位数据
    {
        DHT11_Read_Byte_Fast_Pro();
    }

    for (unsigned short i: timeBuf)
    {
        if (i > timeMax)
        {
            timeMax = i;
        }
        else
        {
            timeMin = i;
        }
    }
    std_delay_25us();
    std_delay_25us();
    DHT11_OUT();
    DHT11_High();


    /**把计数值转为二进制数据*/
    uint16_t timeMed = (timeMax + timeMin) >> 1;//整除2，取中位数
    uint8_t data;                               //临时数据
    bool tempBin;                               //临时二进制数据
    for (int i = 0; i < 5; ++i)
    {
        data = 0;//重置
        for (int j = 0; j < 8; ++j)
        {
            data <<= 1;
            //比较计数值，读取二进制数据
            if (timeBuf[i * 8 + j] > timeMed)
            {
                tempBin = true;
            }
            else
            {
                tempBin = false;
            }
            data |= tempBin;
        }
        buf[i] = data;//存储数据
    }

    /**检验**/
    if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
    {
        humi = (float) (buf[0] + buf[1] * 0.1);
        temp = (float) (buf[2] + buf[3] * 0.1);
        return true;
    }
    else
    {
        return false;
    }
}
#endif

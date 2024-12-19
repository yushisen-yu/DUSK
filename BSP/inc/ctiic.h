#ifndef __MYCT_IIC_H
#define __MYCT_IIC_H

#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
//CT_SCL    PB10
//CT_SDA    PB15

//IO方向设置
#define CT_SDA_IN()  {GPIOB->MODER&=~(3<<(2*15));GPIOB->MODER|=0<<2*15;}    //PB15输入模式
#define CT_SDA_OUT() {GPIOB->MODER&=~(3<<(2*15));GPIOB->MODER|=1<<2*15;}    //PB15输出模式

//IO操作函数
#define CT_IIC_SCL_HIGH  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define CT_IIC_SCL_LOW   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define CT_IIC_SDA_HIGH  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define CT_IIC_SDA_LOW   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define CT_READ_SDA HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)


//IIC所有操作函数
void CT_IIC_Init(void);                    //初始化IIC的IO口
void CT_IIC_Start(void);                //发送IIC开始信号
void CT_IIC_Stop(void);                    //发送IIC停止信号
void CT_IIC_Send_Byte(uint8_t txd);            //IIC发送一个字节
uint8_t CT_IIC_Read_Byte(unsigned char ack);    //IIC读取一个字节
uint8_t CT_IIC_Wait_Ack(void);                //IIC等待ACK信号
void CT_IIC_Ack(void);                    //IIC发送ACK信号
void CT_IIC_NAck(void);                    //IIC不发送ACK信号
#ifdef __cplusplus
}
#endif
#endif








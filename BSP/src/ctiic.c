#include "ctiic.h"

//#define TIM7_CNT (*(volatile uint32_t *)(TIM7_BASE+0x24))
////注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
//void delay_us(uint32_t us)
//{
//    // 频率为84MHz，84M/1000 = 84000
//    uint32_t final_count =TIM7_CNT+us*84000;
//
//
//    // 针对溢出情况
//    while (TIM7_CNT>final_count);
//
//    // 非溢出
//    while (TIM7_CNT < final_count);
//
//}

//控制I2C速度的延时
void CT_Delay(void)
{
//    delay_us(2);
}

//电容触摸芯片IIC接口初始化
void CT_IIC_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;

    //PB10设置为推挽输出
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIOB->MODER =

            //PB15设置为推挽输出
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//产生IIC起始信号
void CT_IIC_Start(void)
{
    CT_SDA_OUT();     //sda线输出
    CT_IIC_SDA_HIGH;
    CT_IIC_SCL_HIGH;
    HAL_Delay(30);
    CT_IIC_SDA_LOW; //START:when CLK is high,DATA change form high to low
    CT_Delay();
    CT_IIC_SCL_LOW; //钳住I2C总线，准备发送或接收数据
}

//产生IIC停止信号
void CT_IIC_Stop(void)
{
    CT_SDA_OUT();//sda线输出
    CT_IIC_SCL_HIGH;
    HAL_Delay(30);
    CT_IIC_SDA_LOW; //STOP:when CLK is high DATA change form low to high
    CT_Delay();
    CT_IIC_SDA_HIGH; //发送I2C总线结束信号
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t CT_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    CT_SDA_IN();      //SDA设置为输入
    CT_IIC_SDA_HIGH;
    CT_IIC_SCL_HIGH;
    CT_Delay();
    while (CT_READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            CT_IIC_Stop();
            return 1;
        }
        CT_Delay();
    }
    CT_IIC_SCL_LOW; //时钟输出0
    return 0;
}

//产生ACK应答
void CT_IIC_Ack(void)
{
    CT_IIC_SCL_LOW;
    CT_SDA_OUT();
    CT_Delay();
    CT_IIC_SDA_LOW;
    CT_Delay();
    CT_IIC_SCL_HIGH;
    CT_Delay();
    CT_IIC_SCL_LOW;
}

//不产生ACK应答
void CT_IIC_NAck(void)
{
    CT_IIC_SCL_LOW;
    CT_SDA_OUT();
    CT_Delay();
    CT_IIC_SDA_HIGH;
    CT_Delay();
    CT_IIC_SCL_HIGH;
    CT_Delay();
    CT_IIC_SCL_LOW;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void CT_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    CT_SDA_OUT();
    CT_IIC_SCL_LOW; //拉低时钟开始数据传输
    CT_Delay();
    for (t = 0; t < 8; t++)
    {
//        CT_IIC_SDA = (txd & 0x80) >> 7;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (txd & 0x80) >> 7);
        txd <<= 1;
        CT_IIC_SCL_HIGH;
        CT_Delay();
        CT_IIC_SCL_LOW;
        CT_Delay();
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t CT_IIC_Read_Byte(unsigned char ack)
{
    uint8_t i, receive = 0;
    CT_SDA_IN();//SDA设置为输入
    HAL_Delay(30);
    for (i = 0; i < 8; i++)
    {
        CT_IIC_SCL_LOW;
        CT_Delay();
        CT_IIC_SCL_HIGH;
        receive <<= 1;
        if (CT_READ_SDA)receive++;
    }
    if (!ack)CT_IIC_NAck();//发送nACK
    else CT_IIC_Ack(); //发送ACK
    return receive;
}





























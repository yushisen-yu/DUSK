#include "adxl345.h"
#include "delay.h"
#include  <math.h>

#define DEVICE_ID        0X00    //器件ID,0XE5
#define THRESH_TAP        0X1D    //敲击阀值
#define OFSX            0X1E
#define OFSY            0X1F
#define OFSZ            0X20
#define DUR                0X21
#define Latent            0X22
#define Window        0X23
#define THRESH_ACK        0X24
#define THRESH_INACT    0X25
#define TIME_INACT        0X26
#define ACT_INACT_CTL    0X27
#define THRESH_FF        0X28
#define TIME_FF            0X29
#define TAP_AXES        0X2A
#define ACT_TAP_STATUS  0X2B
#define BW_RATE            0X2C
#define POWER_CTL        0X2D

#define INT_ENABLE        0X2E
#define INT_MAP            0X2F
#define INT_SOURCE    0X30
#define DATA_FORMAT        0X31
#define DATA_X0            0X32
#define DATA_X1            0X33
#define DATA_Y0            0X34
#define DATA_Y1            0X35
#define DATA_Z0            0X36
#define DATA_Z1            0X37
#define FIFO_CTL        0X38
#define FIFO_STATUS        0X39

//#define     NOP             0xFF    // Define No Operation, might be used to read status register
#define     NOP             0    // Define No Operation, might be used to read status register


/*********************************************************************************************************	
//ADXL345硬件资源引脚定义
//ADXL345_CS(PF14)     OUT
//ADXL345_CLK(PF11)    OUT
//ADXL345_DIN(PF12)    OUT
//ADXL345_DO(PF13)     IN
*********************************************************************************************************/
//ADXL345_CS(PF14)     OUT
#define    ADXL345_CS_L          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,  GPIO_PIN_RESET)
#define    ADXL345_CS_H          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,  GPIO_PIN_SET)

//ADXL345_CLK(PF11)    OUT
#define    ADXL345_CLK_L         HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,  GPIO_PIN_RESET)
#define    ADXL345_CLK_H         HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,  GPIO_PIN_SET)

//ADXL345_DIN(PF12)     OUT
#define    ADXL345_DIN_L          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,  GPIO_PIN_RESET)
#define    ADXL345_DIN_H          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,  GPIO_PIN_SET)

//ADXL345_DO(PF13)      IN
#define    ADXL345_DO          HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)


/********************************内部接口声明*************************************/
void spi_clk(void);

void ADXL345_ISP_Init(void);

unsigned char SPI_RW_Byte(unsigned char dat);

void ADXL345WriteReg(unsigned char addr, unsigned char val);

unsigned char ADXL345ReadReg(unsigned char addr);


/*************************************************************************
函数名称：发送端初始化函数
输入参数：无
输出参数：无
函数返回：无
//ADXL345硬件资源引脚定义
//ADXL345_CS(PF14)     OUT
//ADXL345_CLK(PF11)    OUT
//ADXL345_DIN(PF12)    OUT
//ADXL345_DO(PF13)     IN
*************************************************************************/
void ADXL345_ISP_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    //ADXL345_CS(PF14)、ADXL345_SCK(PF11)、//ADXL345_DO(PF12)
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;           //输出模式 //推挽输出
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

    //ADXL345_DIN(PF13)    IN
    GPIO_InitStructure.Pin = GPIO_PIN_13;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;           //输入模式
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

    ADXL345_CS_H;
    ADXL345_CLK_H;
    ADXL345_DIN_H;

}

/*************************************************************************
函数名称：SPI产生时钟
输入参数：无
输出参数：无
函数返回：无
*************************************************************************/
void spi_clk(void)
{
    ADXL345_CLK_H;
    delay_us(20);

    ADXL345_CLK_L;
    delay_us(20);

}

/*************************************************************************
函数名称：SPI读写数据函数
输入参数：写入的数据
输出参数：无
函数返回：读取到的数据
*************************************************************************/
unsigned char SPI_RW_Byte(unsigned char dat)
{
    uint8_t i;
    uint8_t temp = 0;

    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)//位运算，判断最高位是否为1
        {
            ADXL345_DIN_H;
        }
        else
        {
            ADXL345_DIN_L;
        }

        // 将数据左移，为下一个位准备
        dat <<= 1;


        ADXL345_CLK_H; // 时钟高

        temp <<= 1;

        if (ADXL345_DO)
        {
            temp |= 0x01; // 露脕取碌陆1拢卢路诺陆战确位謨
        }

        delay_us(20);

        ADXL345_CLK_L;
        delay_us(20);
    }

    delay_us(20);

    ADXL345_DIN_L;

    return temp;
}


/***************************************************************************
函数名称：发送端写数据函数
输入参数：寄存器地址、数据
输出参数：无
函数返回：寄存器状态
***************************************************************************/
void ADXL345WriteReg(unsigned char addr, unsigned char val)
{
    // 置低CSN，使能SPI传输
    ADXL345_CS_L;
    delay_us(20);

    spi_clk();

    // 写寄存器
    addr &= 0x3F;
    SPI_RW_Byte(addr);

    delay_us(20);

    // 向寄存器写入数据
//    dat1 = val & 0x7F;  
    SPI_RW_Byte(val);

    ADXL345_CLK_H; // 时钟高

    // CSN拉高，完成
    ADXL345_CS_H;

    delay_us(20);
}

/***************************************************************************
函数名称：发送端读取1字节数据函数
输入参数：寄存器地址
输出参数：无
函数返回：读取到的数据
***************************************************************************/
unsigned char ADXL345ReadReg(unsigned char addr)
{
    uint8_t reg_val;

    //置低CSN，使能SPI传输
    ADXL345_CS_L;
    delay_us(20);

    spi_clk();

    addr |= 0x80;

    SPI_RW_Byte(addr);

    //读取寄存器的值
    reg_val = SPI_RW_Byte(NOP);

    ADXL345_CLK_H; // 时钟高

    //CSN拉高，完成
    ADXL345_CS_H;
    delay_us(20);

    return reg_val;
}

/**********************************************************************************************************
函数名称：ADXL345初始化
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
unsigned char ADXL345_Init(void)
{
    ADXL345_ISP_Init();
    uint8_t temp = 0;

    temp = ADXL345ReadReg(DEVICE_ID);     //  读取器件ID   
    if (temp == 0xE5)                        //  器件ID=0xE5
    {
        ADXL345WriteReg(DATA_FORMAT, 0x2B); //  低电平中断输出,13位全分辨率,输出数据右对齐,16g量程
        ADXL345WriteReg(BW_RATE, 0x0A);     //  数据输出速度为100Hz
        ADXL345WriteReg(POWER_CTL, 0x28);   //  链接使能,测量模式
        ADXL345WriteReg(INT_ENABLE, 0x00);  //  不使用中断

        ADXL345WriteReg(OFSX, 0x00);        //  X轴偏移
        ADXL345WriteReg(OFSY, 0x00);        //  Y轴偏移
        ADXL345WriteReg(OFSZ, 0x00);        //  Z轴偏移

        return 0;
    }

    return 1;
}

/**********************************************************************************************************
函数名称：读取加速度数据
输入参数：数据缓冲区
输出参数：无
函数返回：无
**********************************************************************************************************/
//读取ADXL的平均值
//x,y,z:读取10次后取平均值

//void ADXL345ReadAvval(short *x, short *y, short *z,short av_count)
//{
//    short tx = 0, ty = 0, tz = 0;
//    uint8_t i;
//
//    for (i = 0; i < av_count; i++)
//    {
//        ADXL345Read_XYZ(x, y, z);
//
//        HAL_Delay(10);
//
//        tx += (short) *x;
//        ty += (short) *y;
//        tz += (short) *z;
//    }
//
//    *x = tx / av_count;
//    *y = ty / av_count;
//    *z = tz / av_count;
//
//
//}

/**
  * @brief  对从ADXL345传感器读取的数据进行一阶滤波处理，并计算平均值。
  * @param  x: 指向X轴数据的指针。
  * @param  y: 指向Y轴数据的指针。
  * @param  z: 指向Z轴数据的指针。
  * @param  av_count: 平均计算的次数。1次即可很稳定
  * @retval None
  */
// 定义一个平滑因子alpha，该值介于0到1之间。
// alpha越接近1，滤波效果越弱；越接近0，滤波效果越强。
#define ALPHA 0.5f

void ADXL345ReadAvval(short *x, short *y, short *z, short av_count)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;  // 存储累加值用于计算平均值
    float filtered_x = 0, filtered_y = 0, filtered_z = 0;  // 存储每次一阶滤波后的结果

    for (short i = 0; i < av_count; i++)
    {
        short raw_x, raw_y, raw_z;
        ADXL345Read_XYZ(&raw_x, &raw_y, &raw_z);  // 获取原始数据

        // 应用一阶滤波算法
        filtered_x = ALPHA * filtered_x + (1 - ALPHA) * raw_x;
        filtered_y = ALPHA * filtered_y + (1 - ALPHA) * raw_y;
        filtered_z = ALPHA * filtered_z + (1 - ALPHA) * raw_z;

        // 累加滤波后的数据
        sum_x += filtered_x;
        sum_y += filtered_y;
        sum_z += filtered_z;

        if (i < av_count - 1)
        {  // 最后一次读取不需要延时
            HAL_Delay(10);  // 延时10ms等待下一次读取
        }
    }

    // 计算平均值
    *x = (short) (sum_x / av_count);
    *y = (short) (sum_y / av_count);
    *z = (short) (sum_z / av_count);
    if (*x == 127) { *x = 0; }
    if (*y == 127) { *y = 0; }
}

void ADXL345ReadAvval_Once(short *x, short *y, short *z)
{
    float filtered_x = 0, filtered_y = 0, filtered_z = 0;  // 存储每次一阶滤波后的结果
    short raw_x, raw_y, raw_z;
    ADXL345Read_XYZ(&raw_x, &raw_y, &raw_z);  // 获取原始数据

    // 应用一阶滤波算法
    raw_x = (short) ((1 - ALPHA) * raw_x);
    raw_y =  (short) ((1 - ALPHA) * raw_y);
    raw_z = (short) ( (1 - ALPHA) * raw_z);

    // 计算平均值
    *x = (short) (raw_x == 127 ? 0 : raw_x);
    *y = (short) (raw_y == 127 ? 0 : raw_y);
    *z = (short) (raw_z);
}

/**********************************************************************************************************
函数名称：自动校准
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
void ADXL345_AUTO_Adjust(char *xval, char *yval, char *zval)
{
    short tx, ty, tz;
    uint8_t i;
    short offx = 0, offy = 0, offz = 0;

    ADXL345WriteReg(POWER_CTL, 0x00);                                    //  先进入休眠模式.
    HAL_Delay(100);

    ADXL345WriteReg(DATA_FORMAT, 0x2B);                                    //  低电平中断输出,13位全分辨率,输出数据右对齐,16g量程
    ADXL345WriteReg(BW_RATE, 0x0A);                                        //  数据输出速度为100Hz
    ADXL345WriteReg(POWER_CTL, 0x28);                                    //  链接使能,测量模式
    ADXL345WriteReg(INT_ENABLE, 0x00);                                    //  不使用中断

    ADXL345WriteReg(OFSX, 0x00);
    ADXL345WriteReg(OFSY, 0x00);
    ADXL345WriteReg(OFSZ, 0x00);

    HAL_Delay(12);

    for (i = 0; i < 10; i++)
    {
        ADXL345ReadAvval(&tx, &ty, &tz, 50);

        offx += tx;
        offy += ty;
        offz += tz;
    }

    offx /= 10;
    offy /= 10;
    offz /= 10;

    *xval = -offx / 4;
    *yval = -offy / 4;
    *zval = -(offz - 256) / 4;

    ADXL345WriteReg(OFSX, *xval);
    ADXL345WriteReg(OFSY, *yval);
    ADXL345WriteReg(OFSZ, *zval);
}


/**********************************************************************************************************
函数名称：读取三个轴的数据
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
void ADXL345Read_XYZ(short *x, short *y, short *z)
{
    uint8_t x0, y0, z0;
    uint8_t x1, y1, z1;

    x0 = ADXL345ReadReg(DATA_X0);
    y0 = ADXL345ReadReg(DATA_Y0);
    z0 = ADXL345ReadReg(DATA_Z0);

    x1 = ADXL345ReadReg(DATA_X1);
    y1 = ADXL345ReadReg(DATA_Y1);
    z1 = ADXL345ReadReg(DATA_Z1);

    *x = (short) (((short) x1 << 8) + x0);                              // DATA X1为高位有效字节
    *y = (short) (((short) y1 << 8) + y0);                              // DATA Y1为高位有效字节
    *z = (short) (((short) z1 << 8) + z0);                              // DATA Z1为高位有效字节
}

/**********************************************************************************************************
函数名称：计算角度值
输入参数：无
输出参数：无
函数返回：无
函数说明：
x,y,z:x,y,z方向的重力加速度分量(不需要单位,直接数值即可)
dir:要获得的角度.0,与Z轴的角度;1,与X轴的角度;2,与Y轴的角度.
返回值:角度值.单位0.1°.
**********************************************************************************************************/
short ADXL345Get_Angle(float x, float y, float z, unsigned char dir)
{
    float temp;
    float res = 0;
    short value = 0;

    switch (dir)
    {
        case 0:                                                      //  与自然Z轴的角度
            temp = sqrt((x * x + y * y)) / z;
            res = atan(temp);
            break;

        case 1:                                                      //  与自然X轴的角度
            temp = x / sqrt((y * y + z * z));
            res = atan(temp);
            break;

        case 2:                                                      //  与自然Y轴的角度
            temp = y / sqrt((x * x + z * z));
            res = atan(temp);
            break;
    }

    value = res * 1800 / 3.14;

    return value;
}


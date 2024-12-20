//
// Created by fairy on 2024/9/22.
//

#include "lcd.h"

#ifdef USE_LCD

#include "fsmc.h"
#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream6;

// ******************配置属性******************

/**LCD基址*/
//LCD地址结构体
typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
} LCD_TypeDef;
//使用NOR/SRAM的 Bank1.sector4,地址位HADDR[27,26]=11 A11作为数据命令区分线
//注意设置时STM32内部会右移一位对其! 1111 1111 1110=0XFFE
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x00000FFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

//扫描方向定义
#define L2R_U2D  0                                                      // 从左到右,从上到下
#define L2R_D2U  1                                                      // 从左到右,从下到上
#define R2L_U2D  2                                                      // 从右到左,从上到下
#define R2L_D2U  3                                                      // 从右到左,从下到上

#define U2D_L2R  4                                                      // 从上到下,从左到右
#define U2D_R2L  5                                                      // 从上到下,从右到左
#define D2U_L2R  6                                                      // 从下到上,从左到右
#define D2U_R2L  7                                                      // 从下到上,从右到左

#define DFT_SCAN_DIR  R2L_D2U                                           // 默认的扫描方向

//#define DFT_SCAN_DIR  L2R_U2D                                           // 默认的扫描方向

//LCD分辨率设置
#define SSD_HOR_RESOLUTION        800                                        // LCD水平分辨率
#define SSD_VER_RESOLUTION        480                                        // LCD垂直分辨率

//LCD驱动参数设置
#define SSD_HOR_PULSE_WIDTH        1                                        // 水平脉宽
#define SSD_HOR_BACK_PORCH        46                                        // 水平前廊
#define SSD_HOR_FRONT_PORCH        210                                        // 水平后廊

#define SSD_VER_PULSE_WIDTH        1                                        // 垂直脉宽
#define SSD_VER_BACK_PORCH        23                                        // 垂直前廊
#define SSD_VER_FRONT_PORCH        22                                        // 垂直前廊

//如下几个参数，自动计算
#define SSD_HT    (SSD_HOR_RESOLUTION+SSD_HOR_BACK_PORCH+SSD_HOR_FRONT_PORCH)
#define SSD_HPS    (SSD_HOR_BACK_PORCH)
#define SSD_VT    (SSD_VER_RESOLUTION+SSD_VER_BACK_PORCH+SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)


//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

/***********内联函数*************/
static void LCD_WR_REG(volatile uint16_t regval)
{
    regval = regval;// 这个重复语句不知道谁写的，看着就很烂，算了还是不改动了
    LCD->LCD_REG = regval;
}

static void LCD_WR_DATA(volatile uint16_t data)
{
    data = data;                                                        // 使用-O2优化的时候,必须插入的延时
    LCD->LCD_RAM = data;
}

//读LCD数据
//返回值:读到的值
uint16_t LCD_RD_DATA(void)
{
    volatile uint16_t ram;                                                            // 防止被优化
    ram = LCD->LCD_RAM;

    return ram;
}

//写寄存器
//LCD_Reg:寄存器地址
//LCD_RegValue:要写入的数据
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
    LCD->LCD_REG = LCD_Reg;                                                // 写入要写的寄存器序号
    LCD->LCD_RAM = LCD_RegValue;                                        // 写入数据
}

//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
//所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向(具体定义见lcd.h)
//9320/9325/9328/4531/4535/1505/b505/5408/9341/5310/5510/1963等IC已经实际测试
void LCD_Scan_Dir(uint8_t dir)
{
    uint16_t regval = 0;
    uint16_t dirreg = 0;

    if (lcddev.id == 0X5510)                                             // 5510,特殊处理
    {
        switch (dir)
        {
            case L2R_U2D://从左到右,从上到下
                regval |= (0 << 7) | (0 << 6) | (0 << 5);
                break;

            case L2R_D2U://从左到右,从下到上
                regval |= (1 << 7) | (0 << 6) | (0 << 5);
                break;

            case R2L_U2D://从右到左,从上到下
                regval |= (0 << 7) | (1 << 6) | (0 << 5);
                break;

            case R2L_D2U://从右到左,从下到上
                regval |= (1 << 7) | (1 << 6) | (0 << 5);
                break;

            case U2D_L2R://从上到下,从左到右
                regval |= (0 << 7) | (0 << 6) | (1 << 5);
                break;

            case U2D_R2L://从上到下,从右到左
                regval |= (0 << 7) | (1 << 6) | (1 << 5);
                break;

            case D2U_L2R://从下到上,从左到右
                regval |= (1 << 7) | (0 << 6) | (1 << 5);
                break;

            case D2U_R2L://从下到上,从右到左
                regval |= (1 << 7) | (1 << 6) | (1 << 5);
                break;
        }

        if (lcddev.id == 0X5510)
        {
            dirreg = 0X3600;
        }
        if (lcddev.id != 0X5510)
        {
            regval |= 0X08;                                             // 5510不需要BGR
        }

        LCD_WriteReg(dirreg, regval);

        if (lcddev.id == 0X5510)
        {
            LCD_WR_REG(lcddev.setxcmd);
            LCD_WR_DATA(0);
            LCD_WR_REG(lcddev.setxcmd + 1);
            LCD_WR_DATA(0);
            LCD_WR_REG(lcddev.setxcmd + 2);
            LCD_WR_DATA((lcddev.width - 1) >> 8);
            LCD_WR_REG(lcddev.setxcmd + 3);
            LCD_WR_DATA((lcddev.width - 1) & 0XFF);
            LCD_WR_REG(lcddev.setycmd);
            LCD_WR_DATA(0);
            LCD_WR_REG(lcddev.setycmd + 1);
            LCD_WR_DATA(0);
            LCD_WR_REG(lcddev.setycmd + 2);
            LCD_WR_DATA((lcddev.height - 1) >> 8);
            LCD_WR_REG(lcddev.setycmd + 3);
            LCD_WR_DATA((lcddev.height - 1) & 0XFF);
        }
    }
}

//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(uint8_t dir)
{
    if (dir == 0)                                                        // 竖屏
    {
        lcddev.dir = 0;                                                    // 竖屏
        lcddev.width = 240;
        lcddev.height = 320;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 480;
            lcddev.height = 800;
        }
    } else                                                                // 横屏
    {
        lcddev.dir = 1;                                                    // 横屏
        lcddev.width = 320;
        lcddev.height = 240;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 800;
            lcddev.height = 480;
        }
    }
    LCD_Scan_Dir(DFT_SCAN_DIR);                                            // 默认扫描方向
}


/********************************************************************
 * 名称 : LCD_Init9481
 * 功能 : 液晶初始化
 * 输入 : 无
 * 输出 : 无
 ***********************************************************************/
void lcd_init(void)
{
#if LCD_SORTS == 9481
    // 此处实际是ILI93xx
    HAL_Delay(100);                                                    // 	delay 50 ms

    LCD_WR_REG(0XDA00);
    lcddev.id = LCD_RD_DATA();        //读回0X00
    LCD_WR_REG(0XDB00);
    lcddev.id = LCD_RD_DATA();        //读回0X80
    lcddev.id <<= 8;
    LCD_WR_REG(0XDC00);
    lcddev.id |= LCD_RD_DATA();        //读回0X00
    if (lcddev.id == 0x8000)
    {
        lcddev.id = 0x5510; //NT35510读回的ID是8000H,为方便区分,我们强制设置为5510
    }

    // -------调试
//    lcddev.id = 0x5510;

    if (lcddev.id == 0X5510) //如果是这几个IC,则设置WR时序为最快
    {
        //重新配置写时序控制寄存器的时序
        FSMC_Bank1E->BWTR[6] &= ~(0XF << 0); //地址建立时间(ADDSET)清零
        FSMC_Bank1E->BWTR[6] &= ~(0XF << 8); //数据保存时间清零
        FSMC_Bank1E->BWTR[6] |= 3 << 0;        //地址建立时间(ADDSET)为3个HCLK =18ns
        FSMC_Bank1E->BWTR[6] |= 2 << 8;    //数据保存时间(DATAST)为6ns*3个HCLK=18ns
    }

    if (lcddev.id == 0x5510)
    {
        LCD_WriteReg(0xF000, 0x55);
        LCD_WriteReg(0xF001, 0xAA);
        LCD_WriteReg(0xF002, 0x52);
        LCD_WriteReg(0xF003, 0x08);
        LCD_WriteReg(0xF004, 0x01);
        //AVDD Set AVDD 5.2V
        LCD_WriteReg(0xB000, 0x0D);
        LCD_WriteReg(0xB001, 0x0D);
        LCD_WriteReg(0xB002, 0x0D);
        //AVDD ratio
        LCD_WriteReg(0xB600, 0x34);
        LCD_WriteReg(0xB601, 0x34);
        LCD_WriteReg(0xB602, 0x34);
        //AVEE -5.2V
        LCD_WriteReg(0xB100, 0x0D);
        LCD_WriteReg(0xB101, 0x0D);
        LCD_WriteReg(0xB102, 0x0D);
        //AVEE ratio
        LCD_WriteReg(0xB700, 0x34);
        LCD_WriteReg(0xB701, 0x34);
        LCD_WriteReg(0xB702, 0x34);
        //VCL -2.5V
        LCD_WriteReg(0xB200, 0x00);
        LCD_WriteReg(0xB201, 0x00);
        LCD_WriteReg(0xB202, 0x00);
        //VCL ratio
        LCD_WriteReg(0xB800, 0x24);
        LCD_WriteReg(0xB801, 0x24);
        LCD_WriteReg(0xB802, 0x24);
        //VGH 15V (Free pump)
        LCD_WriteReg(0xBF00, 0x01);
        LCD_WriteReg(0xB300, 0x0F);
        LCD_WriteReg(0xB301, 0x0F);
        LCD_WriteReg(0xB302, 0x0F);
        //VGH ratio
        LCD_WriteReg(0xB900, 0x34);
        LCD_WriteReg(0xB901, 0x34);
        LCD_WriteReg(0xB902, 0x34);
        //VGL_REG -10V
        LCD_WriteReg(0xB500, 0x08);
        LCD_WriteReg(0xB501, 0x08);
        LCD_WriteReg(0xB502, 0x08);
        LCD_WriteReg(0xC200, 0x03);
        //VGLX ratio
        LCD_WriteReg(0xBA00, 0x24);
        LCD_WriteReg(0xBA01, 0x24);
        LCD_WriteReg(0xBA02, 0x24);
        //VGMP/VGSP 4.5V/0V
        LCD_WriteReg(0xBC00, 0x00);
        LCD_WriteReg(0xBC01, 0x78);
        LCD_WriteReg(0xBC02, 0x00);
        //VGMN/VGSN -4.5V/0V
        LCD_WriteReg(0xBD00, 0x00);
        LCD_WriteReg(0xBD01, 0x78);
        LCD_WriteReg(0xBD02, 0x00);
        //VCOM
        LCD_WriteReg(0xBE00, 0x00);
        LCD_WriteReg(0xBE01, 0x64);
        //Gamma Setting
        LCD_WriteReg(0xD100, 0x00);
        LCD_WriteReg(0xD101, 0x33);
        LCD_WriteReg(0xD102, 0x00);
        LCD_WriteReg(0xD103, 0x34);
        LCD_WriteReg(0xD104, 0x00);
        LCD_WriteReg(0xD105, 0x3A);
        LCD_WriteReg(0xD106, 0x00);
        LCD_WriteReg(0xD107, 0x4A);
        LCD_WriteReg(0xD108, 0x00);
        LCD_WriteReg(0xD109, 0x5C);
        LCD_WriteReg(0xD10A, 0x00);
        LCD_WriteReg(0xD10B, 0x81);
        LCD_WriteReg(0xD10C, 0x00);
        LCD_WriteReg(0xD10D, 0xA6);
        LCD_WriteReg(0xD10E, 0x00);
        LCD_WriteReg(0xD10F, 0xE5);
        LCD_WriteReg(0xD110, 0x01);
        LCD_WriteReg(0xD111, 0x13);
        LCD_WriteReg(0xD112, 0x01);
        LCD_WriteReg(0xD113, 0x54);
        LCD_WriteReg(0xD114, 0x01);
        LCD_WriteReg(0xD115, 0x82);
        LCD_WriteReg(0xD116, 0x01);
        LCD_WriteReg(0xD117, 0xCA);
        LCD_WriteReg(0xD118, 0x02);
        LCD_WriteReg(0xD119, 0x00);
        LCD_WriteReg(0xD11A, 0x02);
        LCD_WriteReg(0xD11B, 0x01);
        LCD_WriteReg(0xD11C, 0x02);
        LCD_WriteReg(0xD11D, 0x34);
        LCD_WriteReg(0xD11E, 0x02);
        LCD_WriteReg(0xD11F, 0x67);
        LCD_WriteReg(0xD120, 0x02);
        LCD_WriteReg(0xD121, 0x84);
        LCD_WriteReg(0xD122, 0x02);
        LCD_WriteReg(0xD123, 0xA4);
        LCD_WriteReg(0xD124, 0x02);
        LCD_WriteReg(0xD125, 0xB7);
        LCD_WriteReg(0xD126, 0x02);
        LCD_WriteReg(0xD127, 0xCF);
        LCD_WriteReg(0xD128, 0x02);
        LCD_WriteReg(0xD129, 0xDE);
        LCD_WriteReg(0xD12A, 0x02);
        LCD_WriteReg(0xD12B, 0xF2);
        LCD_WriteReg(0xD12C, 0x02);
        LCD_WriteReg(0xD12D, 0xFE);
        LCD_WriteReg(0xD12E, 0x03);
        LCD_WriteReg(0xD12F, 0x10);
        LCD_WriteReg(0xD130, 0x03);
        LCD_WriteReg(0xD131, 0x33);
        LCD_WriteReg(0xD132, 0x03);
        LCD_WriteReg(0xD133, 0x6D);
        LCD_WriteReg(0xD200, 0x00);
        LCD_WriteReg(0xD201, 0x33);
        LCD_WriteReg(0xD202, 0x00);
        LCD_WriteReg(0xD203, 0x34);
        LCD_WriteReg(0xD204, 0x00);
        LCD_WriteReg(0xD205, 0x3A);
        LCD_WriteReg(0xD206, 0x00);
        LCD_WriteReg(0xD207, 0x4A);
        LCD_WriteReg(0xD208, 0x00);
        LCD_WriteReg(0xD209, 0x5C);
        LCD_WriteReg(0xD20A, 0x00);

        LCD_WriteReg(0xD20B, 0x81);
        LCD_WriteReg(0xD20C, 0x00);
        LCD_WriteReg(0xD20D, 0xA6);
        LCD_WriteReg(0xD20E, 0x00);
        LCD_WriteReg(0xD20F, 0xE5);
        LCD_WriteReg(0xD210, 0x01);
        LCD_WriteReg(0xD211, 0x13);
        LCD_WriteReg(0xD212, 0x01);
        LCD_WriteReg(0xD213, 0x54);
        LCD_WriteReg(0xD214, 0x01);
        LCD_WriteReg(0xD215, 0x82);
        LCD_WriteReg(0xD216, 0x01);
        LCD_WriteReg(0xD217, 0xCA);
        LCD_WriteReg(0xD218, 0x02);
        LCD_WriteReg(0xD219, 0x00);
        LCD_WriteReg(0xD21A, 0x02);
        LCD_WriteReg(0xD21B, 0x01);
        LCD_WriteReg(0xD21C, 0x02);
        LCD_WriteReg(0xD21D, 0x34);
        LCD_WriteReg(0xD21E, 0x02);
        LCD_WriteReg(0xD21F, 0x67);
        LCD_WriteReg(0xD220, 0x02);
        LCD_WriteReg(0xD221, 0x84);
        LCD_WriteReg(0xD222, 0x02);
        LCD_WriteReg(0xD223, 0xA4);
        LCD_WriteReg(0xD224, 0x02);
        LCD_WriteReg(0xD225, 0xB7);
        LCD_WriteReg(0xD226, 0x02);
        LCD_WriteReg(0xD227, 0xCF);
        LCD_WriteReg(0xD228, 0x02);
        LCD_WriteReg(0xD229, 0xDE);
        LCD_WriteReg(0xD22A, 0x02);
        LCD_WriteReg(0xD22B, 0xF2);
        LCD_WriteReg(0xD22C, 0x02);
        LCD_WriteReg(0xD22D, 0xFE);
        LCD_WriteReg(0xD22E, 0x03);
        LCD_WriteReg(0xD22F, 0x10);
        LCD_WriteReg(0xD230, 0x03);
        LCD_WriteReg(0xD231, 0x33);
        LCD_WriteReg(0xD232, 0x03);
        LCD_WriteReg(0xD233, 0x6D);
        LCD_WriteReg(0xD300, 0x00);
        LCD_WriteReg(0xD301, 0x33);
        LCD_WriteReg(0xD302, 0x00);
        LCD_WriteReg(0xD303, 0x34);
        LCD_WriteReg(0xD304, 0x00);
        LCD_WriteReg(0xD305, 0x3A);
        LCD_WriteReg(0xD306, 0x00);
        LCD_WriteReg(0xD307, 0x4A);
        LCD_WriteReg(0xD308, 0x00);
        LCD_WriteReg(0xD309, 0x5C);
        LCD_WriteReg(0xD30A, 0x00);

        LCD_WriteReg(0xD30B, 0x81);
        LCD_WriteReg(0xD30C, 0x00);
        LCD_WriteReg(0xD30D, 0xA6);
        LCD_WriteReg(0xD30E, 0x00);
        LCD_WriteReg(0xD30F, 0xE5);
        LCD_WriteReg(0xD310, 0x01);
        LCD_WriteReg(0xD311, 0x13);
        LCD_WriteReg(0xD312, 0x01);
        LCD_WriteReg(0xD313, 0x54);
        LCD_WriteReg(0xD314, 0x01);
        LCD_WriteReg(0xD315, 0x82);
        LCD_WriteReg(0xD316, 0x01);
        LCD_WriteReg(0xD317, 0xCA);
        LCD_WriteReg(0xD318, 0x02);
        LCD_WriteReg(0xD319, 0x00);
        LCD_WriteReg(0xD31A, 0x02);
        LCD_WriteReg(0xD31B, 0x01);
        LCD_WriteReg(0xD31C, 0x02);
        LCD_WriteReg(0xD31D, 0x34);
        LCD_WriteReg(0xD31E, 0x02);
        LCD_WriteReg(0xD31F, 0x67);
        LCD_WriteReg(0xD320, 0x02);
        LCD_WriteReg(0xD321, 0x84);
        LCD_WriteReg(0xD322, 0x02);
        LCD_WriteReg(0xD323, 0xA4);
        LCD_WriteReg(0xD324, 0x02);
        LCD_WriteReg(0xD325, 0xB7);
        LCD_WriteReg(0xD326, 0x02);
        LCD_WriteReg(0xD327, 0xCF);
        LCD_WriteReg(0xD328, 0x02);
        LCD_WriteReg(0xD329, 0xDE);
        LCD_WriteReg(0xD32A, 0x02);
        LCD_WriteReg(0xD32B, 0xF2);
        LCD_WriteReg(0xD32C, 0x02);
        LCD_WriteReg(0xD32D, 0xFE);
        LCD_WriteReg(0xD32E, 0x03);
        LCD_WriteReg(0xD32F, 0x10);
        LCD_WriteReg(0xD330, 0x03);
        LCD_WriteReg(0xD331, 0x33);
        LCD_WriteReg(0xD332, 0x03);
        LCD_WriteReg(0xD333, 0x6D);
        LCD_WriteReg(0xD400, 0x00);
        LCD_WriteReg(0xD401, 0x33);
        LCD_WriteReg(0xD402, 0x00);
        LCD_WriteReg(0xD403, 0x34);
        LCD_WriteReg(0xD404, 0x00);
        LCD_WriteReg(0xD405, 0x3A);
        LCD_WriteReg(0xD406, 0x00);
        LCD_WriteReg(0xD407, 0x4A);
        LCD_WriteReg(0xD408, 0x00);
        LCD_WriteReg(0xD409, 0x5C);
        LCD_WriteReg(0xD40A, 0x00);
        LCD_WriteReg(0xD40B, 0x81);

        LCD_WriteReg(0xD40C, 0x00);
        LCD_WriteReg(0xD40D, 0xA6);
        LCD_WriteReg(0xD40E, 0x00);
        LCD_WriteReg(0xD40F, 0xE5);
        LCD_WriteReg(0xD410, 0x01);
        LCD_WriteReg(0xD411, 0x13);
        LCD_WriteReg(0xD412, 0x01);
        LCD_WriteReg(0xD413, 0x54);
        LCD_WriteReg(0xD414, 0x01);
        LCD_WriteReg(0xD415, 0x82);
        LCD_WriteReg(0xD416, 0x01);
        LCD_WriteReg(0xD417, 0xCA);
        LCD_WriteReg(0xD418, 0x02);
        LCD_WriteReg(0xD419, 0x00);
        LCD_WriteReg(0xD41A, 0x02);
        LCD_WriteReg(0xD41B, 0x01);
        LCD_WriteReg(0xD41C, 0x02);
        LCD_WriteReg(0xD41D, 0x34);
        LCD_WriteReg(0xD41E, 0x02);
        LCD_WriteReg(0xD41F, 0x67);
        LCD_WriteReg(0xD420, 0x02);
        LCD_WriteReg(0xD421, 0x84);
        LCD_WriteReg(0xD422, 0x02);
        LCD_WriteReg(0xD423, 0xA4);
        LCD_WriteReg(0xD424, 0x02);
        LCD_WriteReg(0xD425, 0xB7);
        LCD_WriteReg(0xD426, 0x02);
        LCD_WriteReg(0xD427, 0xCF);
        LCD_WriteReg(0xD428, 0x02);
        LCD_WriteReg(0xD429, 0xDE);
        LCD_WriteReg(0xD42A, 0x02);
        LCD_WriteReg(0xD42B, 0xF2);
        LCD_WriteReg(0xD42C, 0x02);
        LCD_WriteReg(0xD42D, 0xFE);
        LCD_WriteReg(0xD42E, 0x03);
        LCD_WriteReg(0xD42F, 0x10);
        LCD_WriteReg(0xD430, 0x03);
        LCD_WriteReg(0xD431, 0x33);
        LCD_WriteReg(0xD432, 0x03);
        LCD_WriteReg(0xD433, 0x6D);
        LCD_WriteReg(0xD500, 0x00);
        LCD_WriteReg(0xD501, 0x33);
        LCD_WriteReg(0xD502, 0x00);
        LCD_WriteReg(0xD503, 0x34);
        LCD_WriteReg(0xD504, 0x00);
        LCD_WriteReg(0xD505, 0x3A);
        LCD_WriteReg(0xD506, 0x00);
        LCD_WriteReg(0xD507, 0x4A);
        LCD_WriteReg(0xD508, 0x00);
        LCD_WriteReg(0xD509, 0x5C);
        LCD_WriteReg(0xD50A, 0x00);
        LCD_WriteReg(0xD50B, 0x81);

        LCD_WriteReg(0xD50C, 0x00);
        LCD_WriteReg(0xD50D, 0xA6);
        LCD_WriteReg(0xD50E, 0x00);
        LCD_WriteReg(0xD50F, 0xE5);
        LCD_WriteReg(0xD510, 0x01);
        LCD_WriteReg(0xD511, 0x13);
        LCD_WriteReg(0xD512, 0x01);
        LCD_WriteReg(0xD513, 0x54);
        LCD_WriteReg(0xD514, 0x01);
        LCD_WriteReg(0xD515, 0x82);
        LCD_WriteReg(0xD516, 0x01);
        LCD_WriteReg(0xD517, 0xCA);
        LCD_WriteReg(0xD518, 0x02);
        LCD_WriteReg(0xD519, 0x00);
        LCD_WriteReg(0xD51A, 0x02);
        LCD_WriteReg(0xD51B, 0x01);
        LCD_WriteReg(0xD51C, 0x02);
        LCD_WriteReg(0xD51D, 0x34);
        LCD_WriteReg(0xD51E, 0x02);
        LCD_WriteReg(0xD51F, 0x67);
        LCD_WriteReg(0xD520, 0x02);
        LCD_WriteReg(0xD521, 0x84);
        LCD_WriteReg(0xD522, 0x02);
        LCD_WriteReg(0xD523, 0xA4);
        LCD_WriteReg(0xD524, 0x02);
        LCD_WriteReg(0xD525, 0xB7);
        LCD_WriteReg(0xD526, 0x02);
        LCD_WriteReg(0xD527, 0xCF);
        LCD_WriteReg(0xD528, 0x02);
        LCD_WriteReg(0xD529, 0xDE);
        LCD_WriteReg(0xD52A, 0x02);
        LCD_WriteReg(0xD52B, 0xF2);
        LCD_WriteReg(0xD52C, 0x02);
        LCD_WriteReg(0xD52D, 0xFE);
        LCD_WriteReg(0xD52E, 0x03);
        LCD_WriteReg(0xD52F, 0x10);
        LCD_WriteReg(0xD530, 0x03);
        LCD_WriteReg(0xD531, 0x33);
        LCD_WriteReg(0xD532, 0x03);
        LCD_WriteReg(0xD533, 0x6D);
        LCD_WriteReg(0xD600, 0x00);
        LCD_WriteReg(0xD601, 0x33);
        LCD_WriteReg(0xD602, 0x00);
        LCD_WriteReg(0xD603, 0x34);
        LCD_WriteReg(0xD604, 0x00);
        LCD_WriteReg(0xD605, 0x3A);
        LCD_WriteReg(0xD606, 0x00);
        LCD_WriteReg(0xD607, 0x4A);
        LCD_WriteReg(0xD608, 0x00);
        LCD_WriteReg(0xD609, 0x5C);
        LCD_WriteReg(0xD60A, 0x00);
        LCD_WriteReg(0xD60B, 0x81);

        LCD_WriteReg(0xD60C, 0x00);
        LCD_WriteReg(0xD60D, 0xA6);
        LCD_WriteReg(0xD60E, 0x00);
        LCD_WriteReg(0xD60F, 0xE5);
        LCD_WriteReg(0xD610, 0x01);
        LCD_WriteReg(0xD611, 0x13);
        LCD_WriteReg(0xD612, 0x01);
        LCD_WriteReg(0xD613, 0x54);
        LCD_WriteReg(0xD614, 0x01);
        LCD_WriteReg(0xD615, 0x82);
        LCD_WriteReg(0xD616, 0x01);
        LCD_WriteReg(0xD617, 0xCA);
        LCD_WriteReg(0xD618, 0x02);
        LCD_WriteReg(0xD619, 0x00);
        LCD_WriteReg(0xD61A, 0x02);
        LCD_WriteReg(0xD61B, 0x01);
        LCD_WriteReg(0xD61C, 0x02);
        LCD_WriteReg(0xD61D, 0x34);
        LCD_WriteReg(0xD61E, 0x02);
        LCD_WriteReg(0xD61F, 0x67);
        LCD_WriteReg(0xD620, 0x02);
        LCD_WriteReg(0xD621, 0x84);
        LCD_WriteReg(0xD622, 0x02);
        LCD_WriteReg(0xD623, 0xA4);
        LCD_WriteReg(0xD624, 0x02);
        LCD_WriteReg(0xD625, 0xB7);
        LCD_WriteReg(0xD626, 0x02);
        LCD_WriteReg(0xD627, 0xCF);
        LCD_WriteReg(0xD628, 0x02);
        LCD_WriteReg(0xD629, 0xDE);
        LCD_WriteReg(0xD62A, 0x02);
        LCD_WriteReg(0xD62B, 0xF2);
        LCD_WriteReg(0xD62C, 0x02);
        LCD_WriteReg(0xD62D, 0xFE);
        LCD_WriteReg(0xD62E, 0x03);
        LCD_WriteReg(0xD62F, 0x10);
        LCD_WriteReg(0xD630, 0x03);
        LCD_WriteReg(0xD631, 0x33);
        LCD_WriteReg(0xD632, 0x03);
        LCD_WriteReg(0xD633, 0x6D);
        //LV2 Page 0 enable
        LCD_WriteReg(0xF000, 0x55);
        LCD_WriteReg(0xF001, 0xAA);
        LCD_WriteReg(0xF002, 0x52);
        LCD_WriteReg(0xF003, 0x08);
        LCD_WriteReg(0xF004, 0x00);
        //Display control
        LCD_WriteReg(0xB100, 0xCC);
        LCD_WriteReg(0xB101, 0x00);
        //Source hold time
        LCD_WriteReg(0xB600, 0x05);
        //Gate EQ control
        LCD_WriteReg(0xB700, 0x70);
        LCD_WriteReg(0xB701, 0x70);
        //Source EQ control (Mode 2)
        LCD_WriteReg(0xB800, 0x01);
        LCD_WriteReg(0xB801, 0x03);
        LCD_WriteReg(0xB802, 0x03);
        LCD_WriteReg(0xB803, 0x03);
        //Inversion mode (2-dot)
        LCD_WriteReg(0xBC00, 0x02);
        LCD_WriteReg(0xBC01, 0x00);
        LCD_WriteReg(0xBC02, 0x00);
        //Timing control 4H w/ 4-delay
        LCD_WriteReg(0xC900, 0xD0);
        LCD_WriteReg(0xC901, 0x02);
        LCD_WriteReg(0xC902, 0x50);
        LCD_WriteReg(0xC903, 0x50);
        LCD_WriteReg(0xC904, 0x50);
        LCD_WriteReg(0x3500, 0x00);
        LCD_WriteReg(0x3A00, 0x55); //16-bit/pixel
        LCD_WR_REG(0x1100);
        HAL_Delay(120);
        LCD_WR_REG(0x2900);
    }

    LCD_Display_Dir(0);        //默认为竖屏

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // 打开背光

    LCD_Clear(0xFFFF);


#endif

#if LCD_SORTS == 9488
    // 复位TFT显示屏
    TFT_RST = 1;  // 将TFT复位引脚设为高电平
    HAL_Delay(5);// 等待100毫秒
    TFT_RST = 0;  // 将TFT复位引脚设为低电平
    HAL_Delay(5);// 等待100毫秒
    TFT_RST = 1;  // 将TFT复位引脚设为高电平，完成复位操作
    HAL_Delay(5);// 等待100毫秒

    // 向LCD发送命令退出睡眠模式
    LCD_WRITE_CMD(0x11);// 退出睡眠模式
    HAL_Delay(5);      // 等待100毫秒，确保命令执行完毕

    // 设置LCD电源控制
    LCD_WRITE_CMD(0xd0); // 电源控制命令
    LCD_WRITE_DATA(0x07);// 设置参数
    LCD_WRITE_DATA(0x47);// 设置参数
    LCD_WRITE_DATA(0x19);// 设置参数

    LCD_WRITE_CMD(0xd1); // 电源控制命令
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x36);// 设置参数
    LCD_WRITE_DATA(0x1f);// 设置参数

    LCD_WRITE_CMD(0xd2); // 电源控制命令
    LCD_WRITE_DATA(0x01);// 设置参数
    LCD_WRITE_DATA(0x11);// 设置参数

    // 其他LCD驱动相关设置
    LCD_WRITE_CMD(0xE4); // 驱动模式设置
    LCD_WRITE_DATA(0xa0);// 设置参数

    LCD_WRITE_CMD(0xf3); // 帧速率控制
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x2a);// 设置参数

    LCD_WRITE_CMD(0xc0); // MV偏压控制
    LCD_WRITE_DATA(0x10);// 设置参数
    LCD_WRITE_DATA(0x3b);// 设置参数
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x02);// 设置参数
    LCD_WRITE_DATA(0x11);// 设置参数

    LCD_WRITE_CMD(0xc5); // VCOM控制
    LCD_WRITE_DATA(0x03);// 设置参数

    LCD_WRITE_CMD(0xc8); // Gamma设置
    LCD_WRITE_DATA(0x00);// 设置Gamma参数
    LCD_WRITE_DATA(0x35);// 设置Gamma参数
    LCD_WRITE_DATA(0x23);// 设置Gamma参数
    LCD_WRITE_DATA(0x07);// 设置Gamma参数
    LCD_WRITE_DATA(0x00);// 设置Gamma参数
    LCD_WRITE_DATA(0x04);// 设置Gamma参数
    LCD_WRITE_DATA(0x45);// 设置Gamma参数
    LCD_WRITE_DATA(0x53);// 设置Gamma参数
    LCD_WRITE_DATA(0x77);// 设置Gamma参数
    LCD_WRITE_DATA(0x70);// 设置Gamma参数
    LCD_WRITE_DATA(0x00);// 设置Gamma参数
    LCD_WRITE_DATA(0x04);// 设置Gamma参数

    /**************************设置LCD显示区域****************************/
    LCD_WRITE_CMD(0x0036); // 显示行列设置
    LCD_WRITE_DATA(0x00E9);// 设置参数1110 1001
    /*前两位为调整方向*/
    //默认方向为A9，左下角与右上角 10
    // 29为右下角与左上角  00
    //E9为左上角，右下角  11
    // 69   01

    LCD_WRITE_CMD(0x003a); // RGB信号格式设置
    LCD_WRITE_DATA(0x0055);// 设置参数

    LCD_WRITE_CMD(0x0020);// RAM写入控制

    LCD_WRITE_CMD(0x2a); // 水平地址设置
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x01);// 设置参数
    LCD_WRITE_DATA(0xDF);// 设置参数

    LCD_WRITE_CMD(0x2b); // 垂直地址设置
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x00);// 设置参数
    LCD_WRITE_DATA(0x01);// 设置参数
    LCD_WRITE_DATA(0x3F);// 设置参数

    HAL_Delay(5);      // 等待100毫秒
    LCD_WRITE_CMD(0x29);// 唤醒命令
    LCD_WRITE_CMD(0x2c);// 写入RAM命令
    HAL_Delay(5);      // 等待100毫秒
    TFTLED = 0x01;      // 背光寄存器初始化

    /*我觉得没必要清屏函数*/
//     LCD_Clear(0xFFFF);  // 清除屏幕，设置为白色
#endif
}

void lcd_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *color_p)
{
#ifdef USE_FSMC_DMA
    LCD_Set_Window(x1, y1, x2, y2);//设置LCD屏幕的扫描区域
    HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream6, (uint32_t) color_p, (uint32_t) TFT_DATA_ADDR,
                     ((x2 + 1) - x1) * ((y2 + 1) - y1));
#else
    LCD_Color_Fill(x1, y1, x2, y2, (const uint16_t *)color_p);
#endif
}


#endif
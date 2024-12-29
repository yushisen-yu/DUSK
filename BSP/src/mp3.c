#include "mp3.h"
#include "USART3.h"
#include "stm32f4xx_hal.h"
void setMp3Dev(unsigned char dev)
{
    unsigned char device[5]  = {0x7E, 0x03, 0x09, 0x00, 0xEF};      // 指定设备 0：U 4:FLASH
    device[3] = dev;
    USART6_Senddata(device, 5);
}

void setMp3Vol(unsigned char vol)
{
    unsigned char volume[5]  = {0x7E, 0x03, 0x06, 0x00, 0xEF};      //	音量 0-30
    volume[3] = vol;
    USART6_Senddata(volume, 5);
}

void mp3Play(void)
{
    unsigned char Play[4]  = {0x7E, 0x02, 0x0D, 0xEF};                  //	播放
    USART6_Senddata(Play, 4); 
}

void mp3Stop(void)
{
    unsigned char Stop[4]  = {0x7E, 0x02, 0x10, 0xEF};                  //	停止
    USART6_Senddata(Stop, 4); 
}
void mp3_next()
{
    uint8_t buf[4]={0x7E,0x02,0x01,0xEF};
    USART6_Senddata(buf,4);
}
void mp3_prev()
{
    uint8_t buf[4]={0x7E,0x02,0x02,0xEF};
    USART6_Senddata(buf,4);
}


/**
 * @brief 播放指定序号的mp3文件
 * @param index
 * @note 使用的是大端字节序
 */
void mp3_play_selected(unsigned short index)
{

    unsigned char buf[6]={0x7E,0x04,0x03,0x00,0x00,0xEF};
    buf[3]=index>>8;
    buf[4]=index&0xFF;
//    *(unsigned short*)(&buf[3])=index;// 小端字节序不对

    USART6_Senddata(buf,6);
}


#ifndef __ADXL345_H
#define __ADXL345_H

#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

// 函数声明

unsigned char ADXL345_Init(void);
//unsigned char ADXL345_S_WriteBuf(unsigned char reg , unsigned char *pBuf, unsigned char bytes);
void ADXL345ReadAvval(short *x, short *y, short *z,short av_count);// 获取三轴数据平均10次的结果
void ADXL345ReadAvval_Once(short *x, short *y, short *z);// 使用了一阶滤波算法
void ADXL345Read_XYZ(short *x, short *y, short *z);// 获取三轴数据
void ADXL345_AUTO_Adjust(char *xval, char *yval, char *zval);
short ADXL345Get_Angle(float x, float y, float z, unsigned char dir);

#ifdef __cplusplus
}
#endif
#endif

//
// Created by DUSK on 2024/12/24.
//

#ifndef FURINA_DHT11_H
#define FURINA_DHT11_H


#ifdef __cplusplus
extern "C" {
#endif

void DHT11_init();

//bool DHT11_Read_Data(float &temp,float &humi);
bool DHT11_Read_Data_Fast_Pro(float &temp, float &humi);//自定义的快速读取函数

#ifdef __cplusplus
}
#endif

#endif //FURINA_DHT11_H

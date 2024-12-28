//
// Created by DUSK on 2024/12/28.
//

#ifndef FURINA_BSP_SRC_DCMOTOR_H_
#define FURINA_BSP_SRC_DCMOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

void DCMotor_init();

void DCMotor_forward();// 正转

void DCMotor_reverse();// 反转

void DCMotor_stop();// 停止


#ifdef __cplusplus
}
#endif

#endif //FURINA_BSP_SRC_DCMOTOR_H_

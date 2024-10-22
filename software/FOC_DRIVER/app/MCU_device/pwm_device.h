#ifndef PWM_DEVICE_H
#define PWM_DEVICE_H
#include "main.h"
#include "stdio.h"
#include "tim.h"

/*******************************************************************************
 * @file   motor_control.c
 * @brief  初始化其他电机控制部分的PWM定时器
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
void init_PWM_motor(void);


void set_pwm_channel_1(uint16_t duty);
void set_pwm_channel_2(uint16_t duty);
void set_pwm_channel_3(uint16_t duty);

void set_pwm_channel_1_2(uint16_t duty);
void set_pwm_channel_2_2(uint16_t duty);
void set_pwm_channel_3_2(uint16_t duty);

#endif


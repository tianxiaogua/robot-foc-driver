#include "pwm_device.h"


/*******************************************************************************
 * @file   motor_control.c
 * @brief  初始化其他电机控制部分的PWM定时器
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
void init_PWM_motor()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); 
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    //修改占空比
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    //修改占空比

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); 
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    //修改占空比
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    //修改占空比
}

/*******************************************************************************
 * @file   motor_control.c
 * @brief  设置PMW占空比
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
void set_pwm_channel_1(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); 
}
void set_pwm_channel_2(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); 
}
void set_pwm_channel_3(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty); 
}

/*******************************************************************************
 * @file   motor_control.c
 * @brief  设置PMW占空比
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
void set_pwm_channel_1_2(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); 
}
void set_pwm_channel_2_2(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty); 
}
void set_pwm_channel_3_2(uint16_t duty)
{
	if(duty>3600) duty = 3600;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty); 
}


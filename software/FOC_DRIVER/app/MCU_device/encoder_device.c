#include "encoder_device.h"

Motor_Lift motor_Lift;
Motor_Lift motor_Right;

/*******************************************************************************
 * @file   : motor_control.c
 * @brief  : 编码器定时器初始化
 * @author : Tianxiaogua
 * @date   : 2023-04
 ******************************************************************************/
void encoder_Init(void)
{
	HAL_TIM_Encoder_Start(&encoder_left_htim, TIM_CHANNEL_ALL);      //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&encoder_left_htim,TIM_IT_UPDATE);         	 //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&encoder_left_htim, 10000);                //编码器定时器初始值设定为10000
	
	HAL_TIM_Encoder_Start(&encoder_right_htim, TIM_CHANNEL_ALL);      //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&encoder_right_htim,TIM_IT_UPDATE);         	 //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&encoder_right_htim, 10000);                //编码器定时器初始值设定为10000
	
	HAL_TIM_Base_Start_IT(&htim10);                       //开启10ms定时器中断
	
	motor_Lift.loopNum = 0;                                   //防溢出
	motor_Right.loopNum = 0; 
}


/*******************************************************************************
 * @file   : motor_control.c
 * @brief  : 在定时器中断中计算编码器的转速
 * @author : Tianxiaogua
 * @date   : 2023-04
 ******************************************************************************/
#define parameter_1 9452//参数1 (4* 17 * 139)=9452
#define parameter_2 200//参数1 (100*2)=200
float motor_Lift_his = 0;
float motor_Right_his = 0;
void HAL_TIM_PeriodElapsedCallback_out(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim10.Instance) {
		int16_t pluse_TIM1 = COUNTERNUM_TIM1 - RELOADVALUE_TIM1/2;											
		motor_Lift.totalAngle = pluse_TIM1 + motor_Lift.loopNum * RELOADVALUE_TIM1/2;  
		
		int16_t pluse_TIM2 = COUNTERNUM_TIM2 - RELOADVALUE_TIM2/2;											
		motor_Right.totalAngle = pluse_TIM2 + motor_Right.loopNum * RELOADVALUE_TIM2/2;  
		
		//进行速度计算,根据前文所说的,4倍频,编码器13位,减速比100,再乘以6000即为每分钟输出轴多少转
		motor_Lift.speed = (float)(motor_Lift.totalAngle - motor_Lift.lastAngle)/parameter_1*parameter_2;	// 得到多少转每秒钟
		motor_Lift.lastAngle = motor_Lift.totalAngle;         //更新转过的圈数
		
		motor_Right.speed = (float)(motor_Right.totalAngle - motor_Right.lastAngle)/parameter_1*parameter_2;			
		motor_Right.lastAngle = motor_Right.totalAngle;         //更新转过的圈数
		
		// 在实际的使用过程中会出现不正常的高峰值毛刺，在数据中只会出现一次，某个特别大的数据，正常数据均小于1.8，使用滤波去除
		if(motor_Lift.speed > 1.8f || motor_Lift.speed < -1.8) motor_Lift.speed = motor_Lift_his;
		else motor_Lift_his = motor_Lift.speed;
		if(motor_Right.speed > 1.8f || motor_Right.speed < -1.8) motor_Right.speed = motor_Right_his;
		else motor_Right_his = motor_Right.speed;

		//printf("%.2f %.2f s/r\r\n", 1/motor_Lift.speed, 1/motor_Right.speed);
		//printf("%.2f %.2f rpm\r\n", motor_Lift.speed*60, motor_Right.speed*60);
	} else if(htim->Instance == htim1.Instance) {      //如果是编码器更新中断,即10ms内,脉冲数超过了计数范围,需要进行处理
		if(COUNTERNUM_TIM1 < 10000)	motor_Lift.loopNum++;
		else if(COUNTERNUM_TIM1 > 10000)	motor_Lift.loopNum--;
		__HAL_TIM_SetCounter(&htim1, 10000);             //重新设定初始值			
	} else if(htim->Instance == htim2.Instance) {     //如果是编码器更新中断,即10ms内,脉冲数超过了计数范围,需要进行处理
	
		if(COUNTERNUM_TIM2 < 10000)	motor_Right.loopNum++;
		else if(COUNTERNUM_TIM2 > 10000)	motor_Right.loopNum--;
		__HAL_TIM_SetCounter(&htim2, 10000);             //重新设定初始值			
	}
}

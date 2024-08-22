#ifndef ENCODER_DEVICE_H
#define ENCODER_DEVICE_H
#include "main.h"
#include "stdio.h"
#include "usart.h"
#include "tim.h"
#include "main.h"
#include <stdlib.h>
#include "tim.h"
#include "stdio.h"

#define encoder_left_htim htim3
#define encoder_right_htim htim4

#define FREQUENCY_DOUBLING_4 4 // 单片机的编码器接口4 倍频frequency doubling
#define RELOADVALUE_TIM1 __HAL_TIM_GetAutoreload(&encoder_left_htim)    // 获取自动装载值,本例中为20000
#define COUNTERNUM_TIM1 __HAL_TIM_GetCounter(&encoder_left_htim)        // 获取编码器定时器中的计数值
#define RELOADVALUE_TIM2 __HAL_TIM_GetAutoreload(&encoder_right_htim)   // 获取自动装载值,本例中为20000
#define COUNTERNUM_TIM2 __HAL_TIM_GetCounter(&encoder_right_htim)       // 获取编码器定时器中的计数值

#define RR             139 // 电机减速比 我的是100
#define LINE_NUM        17 // 电机的编码器线数 公司的电机
//#define OEN_SECOND    1000 // 编码器sample interval的采样间隔10ms
#define SAMPLE_INTERVAL 20 // 编码器sample interval的采样间隔10ms
#define ROTA_TIME       60 // 转速Rotational计算的时间 1分钟 60秒


//电机结构体
typedef struct _Motor
{
	int32_t lastAngle;        // 上10ms转过的角度
	int32_t totalAngle;       // 总的角度
	int16_t loopNum;          // 防超上限
	float speed;              // 电机输出轴目前转速,单位为RPM
	float set;
}Motor_Lift , Motor_Right;

extern Motor_Lift motor_Lift;
extern Motor_Lift motor_Right;


void encoder_Init(void);
void HAL_TIM_PeriodElapsedCallback_out(TIM_HandleTypeDef *htim);

#endif

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

#define FREQUENCY_DOUBLING_4 4 // ��Ƭ���ı������ӿ�4 ��Ƶfrequency doubling
#define RELOADVALUE_TIM1 __HAL_TIM_GetAutoreload(&encoder_left_htim)    // ��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM_TIM1 __HAL_TIM_GetCounter(&encoder_left_htim)        // ��ȡ��������ʱ���еļ���ֵ
#define RELOADVALUE_TIM2 __HAL_TIM_GetAutoreload(&encoder_right_htim)   // ��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM_TIM2 __HAL_TIM_GetCounter(&encoder_right_htim)       // ��ȡ��������ʱ���еļ���ֵ

#define RR             139 // ������ٱ� �ҵ���100
#define LINE_NUM        17 // ����ı��������� ��˾�ĵ��
//#define OEN_SECOND    1000 // ������sample interval�Ĳ������10ms
#define SAMPLE_INTERVAL 20 // ������sample interval�Ĳ������10ms
#define ROTA_TIME       60 // ת��Rotational�����ʱ�� 1���� 60��


//����ṹ��
typedef struct _Motor
{
	int32_t lastAngle;        // ��10msת���ĽǶ�
	int32_t totalAngle;       // �ܵĽǶ�
	int16_t loopNum;          // ��������
	float speed;              // ��������Ŀǰת��,��λΪRPM
	float set;
}Motor_Lift , Motor_Right;

extern Motor_Lift motor_Lift;
extern Motor_Lift motor_Right;


void encoder_Init(void);
void HAL_TIM_PeriodElapsedCallback_out(TIM_HandleTypeDef *htim);

#endif

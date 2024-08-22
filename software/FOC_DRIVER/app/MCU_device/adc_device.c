#include "adc_device.h"
#include "main.h"
#include "stdio.h"
//#include "Kalman.h"

struct ADC_DAM_STRUCT S_adc_dma = {0};

/*******************************************************************************
 * @file   : adc_device.c
 * @brief  : 在tim中断中获取一次adc的值，循环读取三次，对应三个通道
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
void get_adc_strat_once(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)S_adc_dma.original_adc_value, 4);
}

/*******************************************************************************
 * @file   : adc_device.c
 * @brief  : DMA读取完成一次ADC后，在中断中获取到ADC的数值
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
void get_ADC_value_storage(void)
{
	S_adc_dma.adc_value[0] = S_adc_dma.original_adc_value[0] - 2048;
	S_adc_dma.adc_value[1] = S_adc_dma.original_adc_value[1] - 2048;
	S_adc_dma.adc_value[2] = S_adc_dma.original_adc_value[2] - 2048;
	S_adc_dma.adc_value[3] = S_adc_dma.original_adc_value[3] - 2048;
	// S_adc_dma.adc_value[2] = S_adc_dma.original_adc_value[2] - 2048;
}

/*******************************************************************************
 * @file   : adc_device.c
 * @brief  : 在foc的算法中读取电机对应的电流值
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
int32_t get_motor_adc(uint8_t Phase)
{
#if 0
	switch (Phase)
	{
		case 0 /* constant-expression */:{
			float temp_value = S_adc_dma.adc_value[0]*1.0f;
			temp_value = KalmanFilter(&kfp, temp_value);
			return (int32_t)temp_value;
		}
		case 1 /* constant-expression */:{
			float temp_value = S_adc_dma.adc_value[1]*1.0f;
			temp_value = KalmanFilter(&kfp1, temp_value);
			return (int32_t)temp_value;
		}
		default:
			break;
	}
	return 0;
#else
	if (Phase >= 4) {
		return 0;
	}
	int32_t value;
	value = (int32_t)(FILTER_KP * (float)S_adc_dma.adc_value[Phase] + (1-FILTER_KP) * (float)S_adc_dma.his_adc_value[Phase]);
	S_adc_dma.his_adc_value[Phase] = value;
	return value;
#endif
}






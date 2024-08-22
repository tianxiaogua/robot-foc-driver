#ifndef ADC_DEVICE_H
#define ADC_DEVICE_H
#include "stdint.h"
#include "adc.h"

//一介低通滤波系数
#define FILTER_KP 0.9f

struct ADC_DAM_STRUCT{
	uint32_t original_adc_value[4]; //DMA原始数据
	int adc_value[4];				//电流实际AD值
	int his_adc_value[4];			//上次电流实际AD值
};

void get_adc_strat_once(void);
void get_ADC_value_storage(void);
int32_t get_motor_adc(uint8_t Phase);

#endif //__Motor1ADC1_H__


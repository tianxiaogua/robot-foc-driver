#ifndef APP_H
#define APP_H


#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "main.h"

#define RECV_OK 0
#define RECV_ERROR -1

typedef struct
{
	float speed_Ma; 
	float speed_Mb; 
}ORDER;

void simpleFOC_init(void);

/*******************************************************************************
 * @file   foc.c
 * @brief  接收数据包 
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
int32_t recv_back_speed(uint8_t *buf, ORDER *recv_order);

/*******************************************************************************
 * @file   foc.c
 * @brief  反馈数据包 
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
void sand_back_speed(float speeda , float speedb);


/*******************************************************************************
 * @file   foc.c
 * @brief  应用函数
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
void app(void);
void app_init(void);
#endif


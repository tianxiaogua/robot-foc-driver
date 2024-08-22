

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

// #include "main.h" 
// #include "stdio.h"

// #include "pwm_device.h"


// #include "MagneticSensor.h" 
// #include "foc_utils.h" 
// #include "FOCMotor.h" 
// #include "BLDCmotor.h" 
// #include "lowpass_filter.h" 
// #include "pid.h"

/******************************************************************************/
#define M1_Enable    HAL_Delay(1);          //高电平使能
#define M1_Disable   HAL_Delay(1);        //低电平失能
/******************************************************************************/
//编码器类型，二者只能选一。设置使用的编码器为1，不使用的为0
#define M1_AS5600    1
#define M1_TLE5012B  0
//本节代码只支持2种编码器，下一节代码支持更多编码器
/******************************************************************************/
#define delay_ms HAL_Delay


#endif


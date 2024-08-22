#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H
//#include "utils.h"

#include "i2c.h"
//#include "STM32bsp.h"
#define AS5600_I2C_HANDLE hi2c1
#define AS5600_I2C_HANDLE2 hi2c2

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

/*
ע��:AS5600�ĵ�ַ0x36��ָ����ԭʼ7λ�豸��ַ,��ST I2C���е��豸��ַ��ָԭʼ�豸��ַ����һλ�õ����豸��ַ
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution 

#define AS5600_RAW_ANGLE_REGISTER  0x0C

#define ROAD 57.295755f
typedef struct
{
  /* data */
  float zero_electric_angle; // 零偏角度 弧度制
  uint8_t motor_pole_pairs;  // 极对数
  float start_angle;         // 刚启动的角度 弧度制带入极对数后的数值
  uint16_t original_data;    // 当前的原始角度
  float radian_now;          // 目前的弧度制角度
  float radian_prev;             // 上次转动的位置
  float full_rotation_offset;    // 转过的整圈数
  float history_all_radian;      // 一共转过的角度
  float last_history_all_radian; // 一共转过的角度
  float shaft_speed; // 轴速度
  float shaft_angle; // 轴角度
}PARAMETER; // parameter

extern PARAMETER motorL_para;
extern PARAMETER motorR_para;



uint32_t bsp_as5600Init(void);
uint32_t bsp_as5600Init2(void);

uint16_t bsp_as5600GetRawAngle(void);
uint16_t bsp_as5600GetRawAngle2(void);

float bsp_as5600GetAngle(void);

float count_shaft_speed(void); // 计算轴速度


// 计算角度
float get_angle(void);
float get_angle2(void);

/*******************************************************************************
 * @brief: 初始化角度的计算
 * @input: motorL_para 电机结构体
 * @input: angle_now 当前的角度，弧度制
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
void init_updata_rotation_parameter(PARAMETER *p_motor_parameter, float angle_now, float zero_electric_angle,float motor_pole_pairs);
/*******************************************************************************
 * @brief: 对外接口，计算速度和角度, 更新转动参数
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
void updata_rotation_parameter(void);
/*******************************************************************************
 * @brief: 对外接口，获取当前的角度 弧度制 0——6.28
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
float get_angle_Lmotor(void);
float get_angle_Rmotor(void);
float get_angle_Lmotor_road(void);
float get_angle_Rmotor_road(void);
/*******************************************************************************
 * @brief: 对外接口，获取当前的转速 弧度制 0——6.28
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
float get_radin_speed_Lmotor(void);
float get_radin_speed_Rmotor(void);

#endif /* __BSP_AS5600_H */



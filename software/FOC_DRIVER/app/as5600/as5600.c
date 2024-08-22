/***
主要参考博主写的驱动:https://blog.csdn.net/xiaoyuanwuhui/article/details/118970127
**/


#include "as5600.h"
#include "usart_device.h"
//#include "kalman.h"
//#include "foc.h"
//#include "rc_filter.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f

static float angle_data_prev; // 上次位置
static float full_rotation_offset; // 转过的整圈数

/*
*i2cWrite
*对IIC底层驱动进行封装
*
**/
static int i2cWrite(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *pData, uint32_t count) 
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  status = HAL_I2C_Master_Transmit(hi2c, dev_addr, pData, count, i2c_time_out); // 对接ST HAL库
  return status;
}


/*
*i2cRead
*对IIC底层驱动进行封装
*
**/
static int i2cRead(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *pData, uint32_t count) 
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  status = HAL_I2C_Master_Receive(hi2c, (dev_addr | 1), pData, count, i2c_time_out); // 对接ST HAL库
  return status;
}


/*
*bsp_as5600Init
*初始化as5600 开始时对角度和圈数设置为0
*
**/
uint32_t bsp_as5600Init(void) 
{
  /* init i2c interface */
  /* init var */
  full_rotation_offset = 0;  // 转过的整圈数
	uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
	i2cWrite(&AS5600_I2C_HANDLE, AS5600_ADDR, &raw_angle_register, 1);
  angle_data_prev = bsp_as5600GetRawAngle(); // 当前的角度
  if(angle_data_prev == 0){
    return 1;
  }
	return 0;
}
uint32_t bsp_as5600Init2(void) 
{
  /* init i2c interface */
  /* init var */
  full_rotation_offset = 0;  // 转过的整圈数
	uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
	i2cWrite(&AS5600_I2C_HANDLE2, AS5600_ADDR, &raw_angle_register, 1);
  angle_data_prev = bsp_as5600GetRawAngle2(); // 当前的角度
  if(angle_data_prev == 0){
    return 1;
  }
	return 0;
}


/*
*i2cRead
*直接从As5600原始角度寄存器读取数据，需要先写入数据后才能读取当前角度
*
**/
uint8_t as5600buffer[2] = {0};
uint16_t bsp_as5600GetRawAngle(void) 
{
  uint16_t raw_angle;

  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  
  i2cWrite(&AS5600_I2C_HANDLE, AS5600_ADDR, &raw_angle_register, 1);
  i2cRead(&AS5600_I2C_HANDLE, AS5600_ADDR, as5600buffer, 2);
  raw_angle = ((uint16_t)as5600buffer[0] << 8) | (uint16_t)as5600buffer[1];
  return raw_angle;
}
uint8_t as5600buffer_[2] = {0};
uint16_t bsp_as5600GetRawAngle2(void) 
{
  uint16_t raw_angle;
  
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  
  i2cWrite(&AS5600_I2C_HANDLE2, AS5600_ADDR, &raw_angle_register, 1);
  i2cRead(&AS5600_I2C_HANDLE2, AS5600_ADDR, as5600buffer_, 2);
  raw_angle = ((uint16_t)as5600buffer_[0] << 8) | (uint16_t)as5600buffer_[1];
  return raw_angle;
}


/**
 * @brief 输出弧度制角度
 * 
 */
float radian; // 弧度
float original_angle;// 原始角度
float get_angle(void)
{
  original_angle = bsp_as5600GetRawAngle();
  radian = (original_angle / (float)AS5600_RESOLUTION)*_2PI;
  return radian;
}
float radian_2;
float original_angle_2;// 原始角度
float get_angle2(void)
{
  original_angle_2 = bsp_as5600GetRawAngle2();
  radian_2 = (original_angle_2 / (float)AS5600_RESOLUTION)*_2PI;
  return radian_2;
}

/*
* 读取转过的弧度 通过计算得到当前已经转过的圈数，如果转速过快，采样频率比较低会出现数据丢失，导致计算圈数不正确
**/
float bsp_as5600GetAngle(void ) 
{
	float rad = 0; // 转过的弧度
	
  float angle_data = original_angle;
  // 跟踪旋转次数
  float d_angle = angle_data - angle_data_prev;
  if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)){
    full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev = angle_data;

	rad = (full_rotation_offset + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
  return rad;
}


float his_rad = 0;
float count_shaft_speed(void)
{
	// 计算转速 2ms为周期计算转速 计算轴速度
	float rad_speed = 0;
  float rad = 0; // 转过的弧度
  float rad_now = bsp_as5600GetAngle(); // 获取角度
  rad = rad_now - his_rad; // 计算角度差
  his_rad = rad_now; // 记录转过的角度
	rad_speed = rad * 500; // speed = rad/2*1000 = rad*500

  // rad_speed  = first_order_filter(rad_speed);
//  rad_speed  = KalmanFilter(&kfp, rad_speed);
	return rad_speed;
}

/// @brief //////////////////////////////////////////////////////////////////////////////////////////


PARAMETER motorL_para;
PARAMETER motorR_para;
/*******************************************************************************
 * @brief: 读取AS5600的原始数据，转换成弧度制
 * @input: motorL_para 电机结构体
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
#define _2PI_AS5600 0.0015339807878857f // _2PI / AS5600_RESOLUTION = _2PI_AS5600
void as5600_resolves_radians(PARAMETER *p_motor)
{
  float recv =0; 
  // float angel_radian = p_motor->original_data * _2PI / AS5600_RESOLUTION; // 原始数据转换为弧度制
  p_motor->shaft_angle = p_motor->original_data * _2PI_AS5600; // 原始数据转换为弧度制
//  recv = _normalizeAngle(( p_motor->shaft_angle - p_motor->zero_electric_angle) * p_motor->motor_pole_pairs);
  p_motor->radian_now = recv;
}


/*******************************************************************************
 * @brief: 读取转过的弧度 通过计算得到当前已经转过的圈数，如果转速过快，采样频率比较低会出现数据丢失，导致计算圈数不正确
 * @input: motorL_para 电机结构体
 * @input: angle_now 当前的角度，弧度制
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
void get_Angle_rotation(PARAMETER *p_motorAngle) 
{
	float rad = 0; // 转过的弧度
	
  float angle_data = p_motorAngle->original_data; // original_angle; 原始角度
  
  float d_angle = angle_data - p_motorAngle->radian_prev; // 跟踪旋转次数
  if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)){
    p_motorAngle->full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
  }
  p_motorAngle->radian_prev = angle_data;

	rad = (p_motorAngle->full_rotation_offset + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
  p_motorAngle->history_all_radian = rad;
}


/*******************************************************************************
 * @brief: 计算轴速度
 * @input: motorL_para 电机结构体
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
//void updata_shaft_speed(PARAMETER *p_motor_speed, Kalman *p_kfp)
//{
//  	// 计算转速 0.5ms为周期计算转速 计算轴速度
//    float rad_speed = 0;
//    float rad = 0; // 转过的弧度
//    rad = p_motor_speed->history_all_radian - p_motor_speed->last_history_all_radian; // 计算角度差
//    p_motor_speed->last_history_all_radian = p_motor_speed->history_all_radian; // 记录转过的角度
//    rad_speed = rad * 2000; // speed = rad*2000 = rad*2000
//    p_motor_speed->shaft_speed = rad_speed;
//}


/*******************************************************************************
 * @brief: 初始化角度的计算
 * @input: motorL_para 电机结构体
 * @input: angle_now 当前的角度，弧度制 同时用于设置电机的零篇角度
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
void init_updata_rotation_parameter(PARAMETER *p_motor_parameter, float angle_now, float zero_electric_angle,float motor_pole_pairs)
{
  p_motor_parameter->zero_electric_angle = zero_electric_angle;
	p_motor_parameter->motor_pole_pairs = motor_pole_pairs;
  //float angle_ = _normalizeAngle(( angle_now - zero_electric_angle) * motor_pole_pairs);
  // angle_ + 0.785399f
  p_motor_parameter->radian_now = 0;
  p_motor_parameter->radian_prev = 0;
//  p_motor_parameter->start_angle = angle_;
}


/*******************************************************************************
 * @brief: 对外接口，计算速度和角度, 更新转动参数
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
void updata_rotation_parameter(void)
{

//  motorL_para.original_data = bsp_as5600GetRawAngle(); // 通信获取原始数值
//  
//  as5600_resolves_radians(&motorL_para); // 计算得到弧度制的角度
//  get_Angle_rotation(&motorL_para); // 计算一共转过的角度
//  updata_shaft_speed(&motorL_para, &kfp); // 更新轴速度

//  motorR_para.original_data = bsp_as5600GetRawAngle2(); // 通信获取原始数值
//  
//  as5600_resolves_radians(&motorR_para); // 计算得到弧度制的角度
//  get_Angle_rotation(&motorR_para); // 计算一共转过的角度
//  updata_shaft_speed(&motorR_para, &kfp2); // 更新轴速度

}


/*******************************************************************************
 * @brief: 对外接口，获取当前的角度 弧度制 0——6.28
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
float get_angle_Lmotor(void)
{
  return motorL_para.radian_now; // 返回转换后的当前角度
}

float get_angle_Rmotor(void)
{
  return motorR_para.radian_now; // 返回转换后的当前角度
}
float get_angle_Lmotor_road(void)
{
  return motorL_para.shaft_angle*ROAD; // 返回转换后的当前角度
}

float get_angle_Rmotor_road(void)
{
  return motorR_para.shaft_angle*ROAD; // 返回转换后的当前角度
}


/**均值滤波*/
float average_filtering(float data, float *buf)
{
  float average;
  for(int i=0; i<4; i++){
    buf[i] = buf[i+1];
    average += buf[i];
  }buf[4] = data;
  average += data;
//  average = average*0.2;
  return average;
}


/*******************************************************************************
 * @brief: 对外接口，获取当前的转速 弧度制 0——6.28
 * @author: tianxiaohua
 * @date: 2024-04
 ******************************************************************************/
float history_Lmotor = 0;
float history_Lmotor_buf[5] = {0};
float history_Rmotor_buf[5] = {0};
float get_radin_speed_Lmotor(void)
{
  return motorL_para.shaft_speed;
  // float rad_speed  = KalmanFilter(&kfp1, motorL_para.shaft_speed); // 卡尔曼滤波
  // float rad_speed = first_order_filter_speed(motorL_para.shaft_speed,history_Lmotor); // 一阶滤波
  // float rad_speed = average_filtering(motorL_para.shaft_speed, history_Lmotor_buf);
  // return rad_speed;
}
float history_Rmotor = 0;
float get_radin_speed_Rmotor(void)
{
  return motorR_para.shaft_speed;
  // float rad_speed  = KalmanFilter(&kfp2, motorR_para.shaft_speed); // 卡尔曼滤波
  // float rad_speed = first_order_filter_speed(motorR_para.shaft_speed,history_Rmotor); // 一阶滤波
  // float rad_speed = average_filtering(motorR_para.shaft_speed, history_Rmotor_buf);
  // return rad_speed;
}


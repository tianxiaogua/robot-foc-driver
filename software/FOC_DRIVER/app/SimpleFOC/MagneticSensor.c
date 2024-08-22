

#include "MyProject.h"
#include "as5600.h"
#include "BLDCMotor.h"
#include "MagneticSensor.h"
/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
// long  cpr;
// float full_rotation_offset;
// long  angle_data_prev;
// unsigned long velocity_calc_timestamp;
// float angle_prev;
/******************************************************************************/

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
#define I2C_getRawCount  bsp_as5600GetRawAngle
#define I2C_getRawCount2  bsp_as5600GetRawAngle2 
/******************************************************************************/

/******************************************************************************/
void MagneticSensor_Init(MOTOR_FOC *motor)
{
	motor->cpr=AS5600_CPR;
	if(motor->motor_name == MOTOR_1)
		motor->angle_data_prev = I2C_getRawCount();  
	if(motor->motor_name == MOTOR_2)
		motor->angle_data_prev = I2C_getRawCount2();  
		
	motor->full_rotation_offset = 0;
	motor->velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(MOTOR_FOC *motor)
{
	float angle_data,d_angle;
	
	if(motor->motor_name == MOTOR_1)
		angle_data = I2C_getRawCount();
	if(motor->motor_name == MOTOR_2)
		angle_data = I2C_getRawCount2();

	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - motor->angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*motor->cpr) ) motor->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	motor->angle_data_prev = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (motor->full_rotation_offset + ( angle_data / (float)motor->cpr) * _2PI) ;
}
/******************************************************************************/

/******************************************************************************/

// float getVelocity_(void)
// {
// 	unsigned long now_us;
// 	float Ts, angle_c, vel;

// 	// calculate sample time
// 	now_us = SysTick->VAL; //_micros();
// 	if(now_us<velocity_calc_timestamp)Ts = (float)(velocity_calc_timestamp - now_us)/9*1e-6;
// 	else
// 		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/9*1e-6;
// 	// quick fix for strange cases (micros overflow)
// 	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

// 	// current angle
// 	angle_c = getAngle(&motor_1);
// 	// velocity calculation
// 	vel = (angle_c - angle_prev)/Ts;

// 	// save variables for future pass
// 	angle_prev = angle_c;
// 	velocity_calc_timestamp = now_us;
// 	return vel;
// }

float get_velocity(MOTOR_FOC *motor)
{
	return motor->tim_velocity_data;
}

void tim_velocity(MOTOR_FOC *motor)
{
	float Ts, angle_c, vel;
	Ts = 0.002; // 500Hz=0.002秒

	// current angle
	angle_c = getAngle(motor);
	// velocity calculation
	vel = (angle_c - motor->angle_prev)/Ts;

	// save variables for future pass
	motor->angle_prev = angle_c;
	motor->tim_velocity_data =  vel;
}


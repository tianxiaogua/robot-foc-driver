

// #include "MyProject.h"
#include "pid.h"
#include "main.h" 
#include "stdio.h"
#include "foc_utils.h" 
/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
// float pid_vel_P, pid_ang_P;
// float pid_vel_I, pid_ang_D;
// float integral_vel_prev;
// float error_vel_prev, error_ang_prev;
// float output_vel_ramp;
// float output_vel_prev;
// unsigned long pid_vel_timestamp, pid_ang_timestamp;

/******************************************************************************/
void PID_init(MOTOR_FOC *motor)
{
	motor->pid.pid_vel_P=0.08;  //0.1
	motor->pid.pid_vel_I=0.1;    //2
	motor->pid.output_vel_ramp=100;       //output derivative limit [volts/second]
	motor->pid.integral_vel_prev=0;
	motor->pid.error_vel_prev=0;
	motor->pid.output_vel_prev=0;
	motor->pid.pid_vel_timestamp=SysTick->VAL;
	
	motor->pid.pid_ang_P=10;
	motor->pid.pid_ang_D=0.5;
	motor->pid.error_ang_prev=0;
	motor->pid.pid_ang_timestamp=SysTick->VAL;
}
/******************************************************************************/
//just P&I is enough,no need D
float PID_velocity(MOTOR_FOC *motor, float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,integral,output;
	float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us< motor->pid.pid_vel_timestamp)Ts = (float)(motor->pid.pid_vel_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + motor->pid.pid_vel_timestamp)/9*1e-6f;
	motor->pid.pid_vel_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = motor->pid.pid_vel_P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = motor->pid.integral_vel_prev + motor->pid.pid_vel_I*Ts*0.5*(error + motor->pid.error_vel_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -motor->voltage_limit, motor->voltage_limit);
	
	// sum all the components
	output = proportional + integral;
	// antiwindup - limit the output variable
	output = _constrain(output, -motor->voltage_limit, motor->voltage_limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - motor->pid.output_vel_prev)/Ts;
	if(output_rate > motor->pid.output_vel_ramp)output = motor->pid.output_vel_prev + motor->pid.output_vel_ramp*Ts;
	else if(output_rate < -motor->pid.output_vel_ramp)output = motor->pid.output_vel_prev - motor->pid.output_vel_ramp*Ts;
	
	// saving for the next pass
	motor->pid.integral_vel_prev = integral;
	motor->pid.output_vel_prev = output;
	motor->pid.error_vel_prev = error;
	
	return output;
}
/******************************************************************************/
//P&D for angle_PID
float PID_angle(MOTOR_FOC *motor, float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,derivative,output;
	//float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us < motor->pid.pid_ang_timestamp)
		Ts = (float)(motor->pid.pid_ang_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + motor->pid.pid_ang_timestamp)/9*1e-6f;
	motor->pid.pid_ang_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = motor->pid.pid_ang_P * error;
	// u_dk = D(ek - ek_1)/Ts
	derivative = motor->pid.pid_ang_D*(error - motor->pid.error_ang_prev)/Ts;
	
	output = proportional + derivative;
	output = _constrain(output, -motor->velocity_limit, motor->velocity_limit);
	
	// limit the acceleration by ramping the output
//	output_rate = (output - output_ang_prev)/Ts;
//	if(output_rate > output_ang_ramp)output = output_ang_prev + output_ang_ramp*Ts;
//	else if(output_rate < -output_ang_ramp)output = output_ang_prev - output_ang_ramp*Ts;
	
	// saving for the next pass
//	output_ang_prev = output;
	motor->pid.error_ang_prev = error;
	
	return output;
}
/******************************************************************************/
/******************************************************************************/


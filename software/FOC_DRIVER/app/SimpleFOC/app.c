#include "app.h"

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "main.h"

#include "stdio.h"
#include "nvic_device.h"
#include "as5600.h"
//#include "STM32bsp.h"
#include "pid.h"
#include "MagneticSensor.h"
#include "pwm_device.h"
#include "usart_device.h"
#include "stdlib.h"
#include "stdio.h"
#include<string.h>


ORDER recv_order;
uint8_t sand_buf[9] = {0};

void simpleFOC_init(void)
{
  motor_1.motor_name = MOTOR_1;
	motor_1.voltage_power_supply=12;   // FOC power
	motor_1.pole_pairs=7;              // Number of motor poles
	motor_1.voltage_limit=6;           // Phase voltage limitation
	motor_1.velocity_limit=20;         //rad/s angleOpenloop() and PID_angle() use it
	motor_1.voltage_sensor_align=2.5;  // 

  motor_2.motor_name = MOTOR_2;
  motor_2.voltage_power_supply=12;   // FOC power
	motor_2.pole_pairs=7;              // Number of motor poles
	motor_2.voltage_limit=6;           // Phase voltage limitation
	motor_2.velocity_limit=20;         //rad/s angleOpenloop() and PID_angle() use it
	motor_2.voltage_sensor_align=2.5;  // 

	// torque_controller=Type_voltage;  //
	// controller=Type_velocity;  //Type_angle; //Type_torque;    //
	
  MagneticSensor_Init(&motor_1);     //AS5600 or TLE5012B
  MagneticSensor_Init(&motor_2);     //AS5600 or TLE5012B
	Motor_init(&motor_1);
  Motor_init(&motor_2);
	Motor_initFOC(&motor_1);
  Motor_initFOC(&motor_2);
	PID_init(&motor_1);                //PID init
  PID_init(&motor_2);                //PID init
  printf("Motor ready.\r\n");
}


/*******************************************************************************
 * @file   foc.c
 * @brief  接收数据包 
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
int32_t recv_back_speed(uint8_t *buf, ORDER *recv_order)
{
	float motor1_speed = 0;
	float motor2_speed = 0;
	/**
	 * 解析数据协议： 
	 * 0  31 数据头
	 *    第一个电机的参数
	 * 1  01 01代表正数 02代表负数
	 * 2  00 整数高八位
	 * 3  00 整数低八位
	 *    第二个电机的参数
	 * 4  02 01代表正数 02代表负数
	 * 5  00 整数高八位
	 * 6  00 整数低八位
	 * 7  00 数据校验和 包括数据头在内的前七位的数据和的最后一个字节
	 * 8  0A 数据包结束
	 * */
	if(Rx2_Len >= 9){
		for(int i=0; i<Rx2_Len; i++){
			if(buf[i] == 0x31 && buf[i+8] == 0x0A){ // 寻找包头和包尾
				uint8_t data_sun=0;
				for(int j=0; j<7; j++) data_sun+= buf[i+j]; // 计算校验和
				if(data_sun != buf[7]) return RECV_ERROR;;
				/** 获取第一个电机的速度参数*/
				uint16_t data1 = buf[i+2]; // 获取高八位
				data1 = (data1<<8) + buf[i+3]; // 获取低八位
				if(buf[i+1] == 0x01) motor1_speed = data1; // 如果是正数
				else if(buf[i+1] == 0x02) motor1_speed = -data1; // 如果是负数
				else motor1_speed = 0; // 其他数代表错误值
				/** 获取第二个电机的速度参数*/
				data1 = buf[i+5]; // 获取高八位
				data1 = (data1<<8) + buf[i+6]; // 获取低八位
				if(buf[i+4] == 0x01) motor2_speed = data1; // 如果是正数
				else if(buf[i+4] == 0x02) motor2_speed = -data1; // 如果是负数
				else motor2_speed = 0; // 其他数代表错误值
				// printf("speed: m1:%f, m2:%f, %x\r\n",motor1_speed, motor2_speed, data_sun);
				// motor1_speed = motor1_speed*0.01*_2PI; // 将n转/秒 转换为 n弧度/秒
				// motor2_speed = motor2_speed*0.01*_2PI;
				motor1_speed = motor1_speed*0.01; // 缩小一百倍
				motor2_speed = motor2_speed*0.01;
				// printf("speed: m1:%f, m2:%f\r\n",motor1_speed, motor2_speed);
				// sand_back_speed(motor1_speed, motor2_speed); // 返回数据
				recv_order->speed_Ma = motor1_speed;
				recv_order->speed_Mb = motor2_speed;
				return RECV_OK;
			}
		}
	}
	return RECV_ERROR;
}


/*******************************************************************************
 * @file   foc.c
 * @brief  反馈数据包 
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
void sand_back_speed(float speeda , float speedb)
{
	// float speed_M1 = -speeda*_2PI_001; // 将n弧度/秒  转换为 n转/秒 扩大一百倍
	// float speed_M2 = speedb*_2PI_001;
	// float speed_M1 = -speeda*100/_2PI; // 将n弧度/秒  转换为 n转/秒 扩大一百倍
	// float speed_M2 = speedb*100/_2PI;
	float speed_M1 = -speeda*100; //扩大一百倍
	float speed_M2 = speedb*100;
	uint16_t speed1 = (uint16_t)fabs(speed_M1);
	uint16_t speed2 = (uint16_t)fabs(speed_M2);
	/**
	 * 解析数据协议： 
	 * 0  31 数据头
	 *    第一个电机的参数
	 * 1  01 01代表正数 02代表负数
	 * 2  00 整数高八位
	 * 3  00 整数低八位
	 *    第二个电机的参数
	 * 4  02 01代表正数 02代表负数
	 * 5  00 整数高八位
	 * 6  00 整数低八位
	 * 7  00 数据校验和 包括数据头在内的前七位的数据和的最后一个字节
	 * 8  0A 数据包结束
	 * */
	sand_buf[0] = 0x31;

	if(speed_M1 >=0) sand_buf[1] = 0x01;
	else sand_buf[1] = 0x02;

	sand_buf[2] = speed1>>8; // 低八位
	sand_buf[3] = speed1; // 高八位
	

	if(speed_M2 >=0) sand_buf[4] = 0x01;
	else sand_buf[4] = 0x02;

	sand_buf[5] = speed2>>8;
	sand_buf[6] = speed2;

	uint8_t data_sun=0;
	for(int i=0; i<7; i++) data_sun+= sand_buf[i]; // 计算校验和
	sand_buf[7] = data_sun;

	sand_buf[8] = 0x0A;

	usart2_driver_Transmit(sand_buf, 9);
}

/*******************************************************************************
 * @file   foc.c
 * @brief  应用函数
 * @author Tianxiaogua
 * @date   2024-04
 ******************************************************************************/
void app(void)
{
	char str[10];
	uint8_t recv_buf[10] = {0};	
	float speed_Ma=0;
	float speed_Mb=0;
	while(1){
		/**
		 * @brief Construct a new if object
		 * 两个串口中断函数用于接收速度控制指令
		 * 串口1 波特率：460800 专门用于调试。可直接发送ASSIC码来控制两个电机的转速
		 * 串口1 返回数据频率和速度环一致：666.66Hz
		 * 串口1 使用ASSIC码直接返回字符转，可以使用VOFA调试工具进行查看
		 */
		if(Rx_Flag){  // Receive flag
			Rx_Flag=0;	// clean flag
			// HAL_UART_Transmit(&huart1, Rx_Buf, Rx_Len, 0xFFFF);
				for(int i=0; i<Rx_Len; i++){
				str[i] = Rx_Buf[i];
			}
			int i=atoi(str);
			printf("%d\r\n",i);
			speed_Ma = i;
      		speed_Mb = -i;
		} 
		
		/**
		 * @brief Construct a new if object
		 * 串口2 波特率115200 用于外部通信，用来控制电机的正反转速，可单独控制两个电机
		 * 串口2 按照固定频率40Hz返回转速数据
		 * 串口2 中断按照40Hz以内的数据接收，对系统干扰比较小
		 */
		if(Rx2_Flag){  // Receive flag
			Rx2_Flag=0;	// clean flag
			memcpy(recv_buf, Rx2_Buf, Rx2_Len);
			//HAL_UART_Transmit(&huart2, Rx2_Buf, Rx2_Len, 0xFFFF);
			int32_t recv = recv_back_speed(recv_buf, &recv_order); // 接受解析指令
			if(recv != RECV_ERROR){ // 设置速度
				speed_Ma = recv_order.speed_Ma;
				speed_Mb = recv_order.speed_Mb;
			}
		} 
		loopFOC(&motor_1, speed_Ma);
		loopFOC(&motor_2, speed_Mb);
    sprintf((char*)send_buf, "D:%f,%f,%f,%f,%f,%f\r\n",
       motor_1.foc.shaft_velocity,
       motor_1.foc.current_sp,
       motor_1.foc.shaft_angle ,
       motor_2.foc.shaft_velocity,
       motor_2.foc.current_sp,
       motor_2.foc.shaft_angle );
    usart_driver_Transmit(send_buf,sizeof(send_buf));
	}
}


void app_init(void)
{
    int recv = bsp_as5600Init();
	if(recv != 0){
		printf("bsp_as5600Init error!!\r\n");
		bsp_as5600Init();
	}
	recv = bsp_as5600Init2();
	if(recv != 0){
		printf("bsp_as5600Init2 error!!\r\n");
		bsp_as5600Init2();
	}
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET); // �ߵ�ƽʹ��DCVоƬ
	printf("%.2f %.2f\r\n",get_angle(), get_angle2());
	
	init_PWM_motor();
  start_interrupt();
	simpleFOC_init();
	app();
}

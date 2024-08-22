
#include "BLDCmotor.h" 
#include "FOCMotor.h"
#include "MagneticSensor.h" 
#include "lowpass_filter.h"
#include "main.h"
#include "stdio.h"
// shaft angle calculation
float shaftAngle(MOTOR_FOC *motor)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return motor->sensor_direction*getAngle(motor) - motor->foc.sensor_offset;
}
// shaft velocity calculation
float shaftVelocity(MOTOR_FOC *motor)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_velocity;
  if(motor->motor_name == MOTOR_1)
    return motor->sensor_direction*LPF_velocity(get_velocity(motor));
  if(motor->motor_name == MOTOR_2)
    return motor->sensor_direction*LPF_velocity2(get_velocity(motor)); // motor->tim_velocity_data从中断中计算得到的轴速度
  while (1)
  {
    printf("motor error\r\n");
    HAL_Delay(1000);
  }
  
  
}
/******************************************************************************/
float electricalAngle(MOTOR_FOC *motor)
{
  return _normalizeAngle((motor->foc.shaft_angle + motor->foc.sensor_offset) * motor->pole_pairs - motor->foc.zero_electric_angle);
}
/******************************************************************************/



#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#include "FOCMotor.h" 
/******************************************************************************/
// extern long  cpr;
// extern float full_rotation_offset;
// extern long  angle_data_prev;
// extern unsigned long velocity_calc_timestamp;
// extern float angle_prev;
/******************************************************************************/
void MagneticSensor_Init(MOTOR_FOC *motor);
float getAngle(MOTOR_FOC *motor);
// float getVelocity(void);
float get_velocity(MOTOR_FOC *motor);
void tim_velocity(MOTOR_FOC *motor);
/******************************************************************************/

#endif

#ifndef PID_H
#define PID_H

#include "BLDCmotor.h" 



/******************************************************************************/
void PID_init(MOTOR_FOC *motor);
float PID_velocity(MOTOR_FOC *motor, float error);
float PID_angle(MOTOR_FOC *motor, float error);
/******************************************************************************/

#endif


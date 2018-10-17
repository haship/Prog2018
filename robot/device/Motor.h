/*
 * Motor.h
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_MOTOR_H_
#define ROBOT_DEVICE_MOTOR_H_

#include "ev3api.h"

typedef struct Motor {
    motor_type_t mType;
    motor_port_t mPort;
} Motor;

void Motor_init(Motor* this, motor_port_t port, motor_type_t type);
int32_t Motor_getAngle(Motor* this);
void Motor_resetAngle(Motor* this);
void Motor_setPower(Motor* this, int power);
int Motor_getPower(Motor* this);
void Motor_stop(Motor* this);

#endif /* ROBOT_DEVICE_MOTOR_H_ */

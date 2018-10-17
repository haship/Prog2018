/*
 * Motor.c
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#include "Motor.h"

void Motor_init(Motor* this, motor_port_t port, motor_type_t type){
    this->mPort = port;
    this->mType = type;
}

int32_t Motor_getAngle(Motor* this){
    return ev3_motor_get_counts(this->mPort);
}

void Motor_resetAngle(Motor* this){
    ev3_motor_reset_counts(this->mPort);
}

void Motor_setPower(Motor* this, int power){
    ev3_motor_set_power(this->mPort, power);
}

int Motor_getPower(Motor* this){
    return ev3_motor_get_power(this->mPort);
}

void Motor_stop(Motor* this){
    ev3_motor_stop(this->mPort, true);
}

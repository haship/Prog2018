/*
 * Battery.h
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_BATTERY_H_
#define ROBOT_DEVICE_BATTERY_H_

#include "ev3api.h"

typedef struct Battery {

} Battery;

void Battery_init(Battery* this);
int Battery_getCurrent(Battery* this);
int Battery_getVoltage(Battery* this);

#endif /* ROBOT_DEVICE_BATTERY_H_ */

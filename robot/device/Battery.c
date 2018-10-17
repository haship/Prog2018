/*
 * Battery.c
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#include "Battery.h"

void Battery_init(Battery* this){
	// なにもしない
}

int Battery_getCurrent(Battery* this){
	return ev3_battery_current_mA();
}

int Battery_getVoltage(Battery* this){
	return ev3_battery_voltage_mV();
}

/*
 * UT_TouchSensor.h
 *
 *  Created on: 2018/10/22
 *      Author: hashi
 */

#ifndef TEST_UT_TOUCHSENSOR_H_
#define TEST_UT_TOUCHSENSOR_H_

#include "../robot/device/TouchSensor.h"

typedef struct UT_TouchSensor{

	TouchSensor* touch;

} UT_TouchSensor;

void UT_TouchSensor_test(UT_TouchSensor* this, TouchSensor* touch);

#endif /* TEST_UT_TOUCHSENSOR_H_ */

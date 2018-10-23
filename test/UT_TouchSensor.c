/*
 * UT_TouchSensor.c
 *
 *  Created on: 2018/10/22
 *      Author: hashi
 */

#include "UT_TouchSensor.h"

void UT_TouchSensor_test(UT_TouchSensor* this, TouchSensor* touch){

	this->touch = touch;

	TouchSensor_init(this->touch, TOUCH_SENSOR);

	while(1){

		if(TouchSensor_isPressed(this->touch)){
			ev3_speaker_play_tone(440, 10);
		}

		tslp_tsk(10);

	}

}

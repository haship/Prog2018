/*
 * LedLight.c
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#include "LedLight.h"

void LedLight_init(LedLight* this, ledcolor_t color){
	LedLight_setColor(this, color);
}

void LedLight_setColor(LedLight* this, ledcolor_t color){
	this->mColor = color;
	ev3_led_set_color(this->mColor);
}


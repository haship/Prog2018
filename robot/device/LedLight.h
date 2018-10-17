/*
 * LedLight.h
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_LEDLIGHT_H_
#define ROBOT_DEVICE_LEDLIGHT_H_

#include "ev3api.h"

typedef struct LedLight {
	ledcolor_t mColor;
} LedLight;

void LedLight_init(LedLight* this, ledcolor_t color);
void LedLight_setColor(LedLight* this, ledcolor_t color);

#endif /* ROBOT_DEVICE_LEDLIGHT_H_ */

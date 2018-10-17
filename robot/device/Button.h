/*
 * Button.h
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_BUTTON_H_
#define ROBOT_DEVICE_BUTTON_H_

#include "ev3api.h"

typedef struct Button {
	button_t mType;
} Button;

void Button_init(Button* this, button_t type);
bool_t Button_isPressed(Button* this);

#endif /* ROBOT_DEVICE_BUTTON_H_ */

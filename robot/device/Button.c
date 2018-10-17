/*
 * Button.c
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#include "Button.h"

void Button_init(Button* this, button_t type){
	this->mType = type;
}

bool_t Button_isPressed(Button* this){
	return ev3_button_is_pressed(this->mType);
}

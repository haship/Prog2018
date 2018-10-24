/*
 * ColorSensor.c
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#include "ColorSensor.h"

void ColorSensor_init(ColorSensor* this, sensor_port_t port){
    this->mPort = port;
    this->mType = COLOR_SENSOR;
    ev3_sensor_config(this->mPort, this->mType);
    this->output = 0.0F;
    ev3_color_sensor_get_reflect(this->mPort);
}

uint8_t ColorSensor_getBrightness(ColorSensor* this){
    this->output = COLOR_LPF_PARAMETER * this->output + (1.0F - COLOR_LPF_PARAMETER) * ev3_color_sensor_get_reflect(this->mPort);
    return (uint8_t)this->output;
}

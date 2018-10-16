/*
 * ColorSensor.h
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_COLORSENSOR_H_
#define ROBOT_DEVICE_COLORSENSOR_H_

#include "ev3api.h"

#define COLOR_LPF_PARAMETER (0.1F)

typedef struct ColorSensor {
    sensor_type_t mType;
    sensor_port_t mPort;
    float output;
} ColorSensor;

void ColorSensor_init(ColorSensor* this, sensor_port_t port);
uint8_t ColorSensor_getBrightness(ColorSensor* this);

#endif /* ROBOT_DEVICE_COLORSENSOR_H_ */

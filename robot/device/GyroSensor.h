/*
 * GyroSensor.h
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_GYROSENSOR_H_
#define ROBOT_DEVICE_GYROSENSOR_H_

#include "ev3api.h"

#define GYRO_OFFSET 0
#define GYRO_LPF_PARAMETER (0.1F)

typedef struct GyroSensor {
  sensor_type_t mType;
  sensor_port_t mPort;
  float output_rate;
  float output_angle;
} GyroSensor;

void GyroSensor_init(GyroSensor* this, sensor_port_t port);
int16_t GyroSensor_getAngle(GyroSensor* this);
int16_t GyroSensor_getRate(GyroSensor* this);
int16_t GyroSensor_getOffset(GyroSensor* this);
void GyroSensor_reset(GyroSensor* this);

#endif /* ROBOT_DEVICE_GYROSENSOR_H_ */

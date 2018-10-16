/*
 * SonarSensor.h
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_SONARSENSOR_H_
#define ROBOT_DEVICE_SONARSENSOR_H_

#include "ev3api.h"

#define SONAR_LPF_PARAMETER (0.1F)

typedef struct SonarSensor {
  sensor_type_t mType;
  sensor_port_t mPort;
  float output;
} SonarSensor;

void SonarSensor_init(SonarSensor* this, sensor_port_t port);
int16_t SonarSensor_getDistance(SonarSensor* this);
bool_t SonarSensor_listen(SonarSensor* this);

#endif /* ROBOT_DEVICE_SONARSENSOR_H_ */

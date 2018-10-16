/*
 * GyroSensor.c
 *
 *  Created on: 2018/10/15
 *      Author: hashi
 */

#include "GyroSensor.h"

void GyroSensor_init(GyroSensor* this, sensor_port_t port){
  this->mPort = port;
  this->mType = GYRO_SENSOR;
  this->output_rate = 0.0F;
  this->output_angle = 0.0F;
  ev3_sensor_config(this->mPort, this->mType);
}

int16_t GyroSensor_getAngle(GyroSensor* this){
    this->output_angle = GYRO_LPF_PARAMETER * this->output_angle + (1.0F - GYRO_LPF_PARAMETER) * ev3_gyro_sensor_get_angle(this->mPort);
    return (int16_t)this->output_angle;
}

int16_t GyroSensor_getRate(GyroSensor* this){
    this->output_rate = GYRO_LPF_PARAMETER * this->output_rate + (1.0F - GYRO_LPF_PARAMETER) * ev3_gyro_sensor_get_rate(this->mPort);
    return (int16_t)this->output_rate;
}

int16_t GyroSensor_getOffset(GyroSensor* this){
    return GYRO_OFFSET;
}

void GyroSensor_reset(GyroSensor* this){
    ev3_gyro_sensor_reset(this->mPort);
}

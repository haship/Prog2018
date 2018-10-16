#include "SonarSensor.h"

void SonarSensor_init(SonarSensor* this, sensor_port_t port){
    this->mPort = port;
    this->mType = ULTRASONIC_SENSOR;
    this->output = 0.0F;
    ev3_sensor_config(this->mPort, this->mType);
}

int16_t SonarSensor_getDistance(SonarSensor* this){
    this->output = SONAR_LPF_PARAMETER * this->output + (1.0F - SONAR_LPF_PARAMETER) * ev3_ultrasonic_sensor_get_distance(this->mPort);
    return (int16_t)(this->output);
}

bool_t SonarSensor_listen(SonarSensor* this){
    return ev3_ultrasonic_sensor_listen(this->mPort);
}
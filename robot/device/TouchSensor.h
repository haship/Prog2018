//TouchSensor.h

#ifndef ROBOT_DEVICE_TOUCHSENSOR_H_
#define ROBOT_DEVICE_TOUCHSENSOR_H_

#include "ev3api.h"
// #include "app.h"

//TouchSensorクラスの属性を構造体で実装
typedef struct TouchSensor {
  sensor_type_t mType;  //センサタイプ（接続するセンサの種類）
  sensor_port_t mPort;  //入力ポート
} TouchSensor;

//TouchSensorクラスの操作をクラス名_関数名(引数)で実装
//ヘッダファイルには他のクラスで使用する操作だけをプロトタイプ宣言する
//他のクラスで使用しない操作はソースファイルで宣言する
void TouchSensor_init(TouchSensor* this, sensor_port_t port);
bool_t TouchSensor_isPressed(TouchSensor* this);

#endif//ROBOT_DEVICE_TOUCHSENSOR_H_

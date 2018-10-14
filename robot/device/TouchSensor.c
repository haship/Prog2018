#include "TouchSensor.h"

/**
 * タッチセンサの初期化処理
 * 
 * 引数の型 引数名           説明
 *  TouchSensor* this       タッチセンサ型のポインタ    this
 *  sensor_port_t port      sensor_port_t型 port
 * 
 * 戻り値　なし
 * 
 */
void TouchSensor_init(TouchSensor* this, sensor_port_t port){
  this->mPort = port;           //入力ポート 引数で指定
  this->mType = TOUCH_SENSOR;   //センサータイプ TOUCH_SENSORを代入
  ev3_sensor_config(this->mPort, this->mType);      //属性値を使ってセンサーポートの設定を実行
}

/**
 * タッチセンサを押したかどうか
 * 
 * 引数型　引数名       説明
 *  TouchSensor* this   タッチセンサ型のポインタ this
 * 
 * 戻り値
 *  true  押されている
 *  false 押されていない
 */
bool_t TouchSensor_isPressed(TouchSensor* this){
  return ev3_touch_sensor_is_pressed(this->mPort);  //指定のセンサーポートのタッチセンサの状態を検出する
}

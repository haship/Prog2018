/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : Lコース走行プログラム
 **
 ** 考察 : ・距離の値が不安定
 **        ・PID制御の値が不安定
 **        ・switch文によるプログラムの遅延の可能性有り
 **        ・一部プログラムの簡略化も可能
 **        ・ロケットスタートの調整が必要
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#include "robot/device/TouchSensor.h"
#include "robot/device/SonarSensor.h"
#include "robot/device/ColorSensor.h"
#include "robot/device/GyroSensor.h"
#include "robot/device/Motor.h"
#include "robot/tech/PIDControl.h"
#include "robot/tech/LowPassFilter.h"
#include "robot/EV3way.h"
#include "strategy/course/Section.h"
#include "strategy/course/Stage.h"
#include "strategy/course/Course.h"
#include "strategy/walker/BalanceWalker.h"
#include "strategy/walker/TailWalker.h"
#include "app/Calibration.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//状態
typedef enum State {
	STATE_INIT = 0,		//初期状態
	STATE_CAL,			//キャリブレーション中
	STATE_READY,		//待機中
	STATE_RUNNING,		//走行中
	STATE_STOP,			//完全停止中
	STATE_FAIL,			//転倒中
	STATE_SAVE,			//走行ログ保存中
	STATE_END,			//終了状態
	STATE_TEST,			//試験中
	TNUM_STATE			//状態の数(一番最後の行に書くこと)
} State;

//イベント
typedef enum Event {
	EVENT_NONE = 0,			//イベント未発生
	EVENT_TOUCH_IS_PRESSED,	//タッチセンサーを押した
	EVENT_BT_START,			//リモートスタートした
	EVENT_BT_STOP,			//リモートストップする
	EVENT_FAIL,				//転倒発生
	TNUM_EVENT,				//イベントの数（一番最後の行に書くこと）
} Event;

//オブジェクトを静的に確保する
TouchSensor	gTouchSensor;		//タッチセンサ
SonarSensor gSonarSensor;		//超音波センサ
ColorSensor gColorSensor;		//カラーセンサー（反射光モード）
GyroSensor  gGyroSensor;		//ジャイロセンサ
Motor 		gLeftMotor;			//左モータ
Motor 		gRightMotor;		//右モータ
Motor 		gTailMotor;			//尻尾モータ

/* EV3システム生成 */
static void user_system_create(){

	//デバイス初期化
	TouchSensor_init(&gTouchSensor, EV3_PORT_1);
	SonarSensor_init(&gSonarSensor, EV3_PORT_2);
	ColorSensor_init(&gColorSensor, EV3_PORT_3);
	GyroSensor_init( &gGyroSensor,  EV3_PORT_4);
	Motor_init(&gLeftMotor,  EV3_PORT_C, LARGE_MOTOR);
	Motor_init(&gRightMotor, EV3_PORT_B, LARGE_MOTOR);
	Motor_init(&gTailMotor,  EV3_PORT_A, LARGE_MOTOR);

	//オブジェクトの初期化とリンク付け


	//初期化完了通知
	ev3_led_set_color(LED_ORANGE);
}

//終了処理
static void user_system_destroy(){
	Motor_stop(&gLeftMotor);
	Motor_stop(&gRightMotor);
	Motor_stop(&gTailMotor);
}

//イベント確認
static Event checkEvent(){

	Event e = EVENT_NONE;

	//イベント発生を確認して、エベント名を返す
	if(TouchSensor_isPressed(&gTouchSensor)) e = EVENT_TOUCH_IS_PRESSED;
	else e = EVENT_NONE;

	return e;
}

//イベントハンドラ
static State handleEvent(State s, Event e){

	State ss = s;	//現在の状態を保存

	switch(ss){
		case STATE_INIT:	//初期状態
			user_system_create();	//初期化処理
			ss = STATE_TEST;		//状態遷移(初期状態->試験中)
			break;

		case STATE_CAL:		//キャリブレーション中
			break;
		case STATE_READY:	//待機中
			break;
		case STATE_RUNNING:	//走行中
			break;
		case STATE_STOP:	//完全停止中
			break;
		case STATE_FAIL:	//転倒中
			break;
		case STATE_SAVE:	//走行ログ保存中
			break;
		case STATE_END:		//終了状態
			user_system_destroy();	//終了処理
			break;

		case STATE_TEST:	//試験中
			if(e == EVENT_TOUCH_IS_PRESSED) {
				ev3_speaker_play_tone(NOTE_C4, 10);
			}

			if(ev3_button_is_pressed(UP_BUTTON)) ss = STATE_END;

			break;

		default:			//エラー処理
			break;
		}

	return ss;	//次の状態を返す
}

/* メインタスク */
void main_task(intptr_t unused)
{
	State state = STATE_INIT;	//状態変数を初期状態に設定
	Event event = EVENT_NONE;	//イベント変数をイベント未発生に設定
	
	while(1){
		event = checkEvent();				//イベント発生を確認、イベント名を返す
		state = handleEvent(state, event);	//現在の状態とイベントを基にして処理実行、次に状態遷移する
		tslp_tsk(4);	//4ms待機
	}
	ext_tsk();

}


//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
 
}


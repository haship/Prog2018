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
#include "app/Calibration.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */

/* sample_c1マクロ */
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  1          /* 黒色の光センサ値 */
#define LIGHT_GRAY  20          /* 灰色の光センサ値 */

/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */

/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP  85 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モーター制御比例係数 */
#define D_GAIN             0.028F /* 完全停止用モーター制御 */
#define PWM_ABS_MAX          60 /* 完全停止用モーター制御PWM絶対最大値 */

static float tail[2] = {0,0}; /* 完全停止用モーター制御 */

/* sample_c4マクロ */
//#define DEVICE_NAME     "ET150"  /* Bluetooth名 sdcard:\ev3rt\etc\rc.conf.ini LocalNameで設定 */
//#define PASS_KEY        "9753" /* パスキー    sdcard:\ev3rt\etc\rc.conf.ini PinCodeで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* PID制御 */
#define KP 0.42
#define KI 0.06
#define KD 0.060

/* ロケットスタート */
#define TAIL_ANGLE_ROCKET_START 88

/* 転倒閾値 */
#define F_DOWN 30
#define B_DOWN -30

/* 走行距離計算 */
#define TIRE_DIAMETER 0.098
#define PI 3.14

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void tail_control(signed int angle);
static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);

/* ルックアップゲート */
#define stop 0
#define LOOK_START_ANGLE 63
#define LOOK_rpwm 20
#define LOOK_lpwm 20
#define LOOK_b_rpwm -20
#define LOOK_b_lpwm -20

#define GARREZI 75

/* EV3システム生成 */
static user_system_create(){

}

/* メインタスク */
void main_task(intptr_t unused)
{
	signed char forward;      /* 前後進命令 */
	unsigned char forward_max; /* foward増し */
	signed char turn;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モーターPWM出力 */

	/* PID制御 */
	float p;
	float i;
	float d;
	float diff[2] = {0,0};
	float integral = 0;

	/* 走行距離計算処理 */
	float distance4msL = 0;
	float distance4msR = 0;
	float distance4ms = 0;
	double distance = 0.0;
	float cur_angleL = 0;
	float cur_angleR = 0;
	float pre_angleL = 0;
	float pre_angleR = 0;

	/* パターン分析 */
	int kyori;

	/* 転倒検知 */
	int tentou = 0;

	/* ロケットスタート */
	int dash;

	/* ルックアップゲート */
	int cnt = 0;/* ルックアップゲート内の全てのfor文で使用 */

	/* LCD画面表示 */
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	ev3_lcd_draw_string("EV3way-ET practice-1", 0, CALIB_FONT_HEIGHT*1);

	/* センサー入力ポートの設定 */
	ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
	ev3_sensor_config(color_sensor, COLOR_SENSOR);
	ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
	ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
	ev3_sensor_config(gyro_sensor, GYRO_SENSOR);

	/* モーター出力ポートの設定 */
	ev3_motor_config(left_motor, LARGE_MOTOR);
	ev3_motor_config(right_motor, LARGE_MOTOR);
	ev3_motor_config(tail_motor, LARGE_MOTOR);
	ev3_motor_reset_counts(tail_motor);

	/* Open Bluetooth file */
	bt = ev3_serial_open_file(EV3_SERIAL_BT);
	assert(bt != NULL);

	/* Bluetooth通信タスクの起動 */
	act_tsk(BT_TASK);

	ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

	/* スタート待機 */

	tslp_tsk(1000);

	while(1)
	{
		tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

		if (bt_cmd == 1)
		{
			break; /* リモートスタート */
		}

		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
		{
			break; /* タッチセンサが押された */
		}

		tslp_tsk(10); /* 10msecウェイト */
	}

	/* ジャイロセンサーリセット */
	ev3_gyro_sensor_reset(gyro_sensor);
	balance_init(); /* 倒立振子API初期化 */

	/* 走行モーターエンコーダーリセット */
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);

	ev3_led_set_color(LED_GREEN); /* スタート通知 */

	for(dash=0;dash<5;dash++)
	{
		tail_control(TAIL_ANGLE_ROCKET_START); /* ロケットスタート */
		tslp_tsk(4); /* 4msecウェイト */
	}

	/*
	* Main loop for the self-balance control algorithm
	*/

	/* 走行スタート */
	while(1)
	{

		/* while文内で１度定義 */
		/* 倒立振子制御APIに使用 */
		int32_t motor_ang_l, motor_ang_r;
		int gyro, volt;

		/* パターン分析 */
		if((distance >= 0)&&(distance <= 2.405))
		{
			kyori = 0;
		}
		else if((distance >= 2.406)&&(distance <= 3.960))
		{
			kyori = 1;
		}
		else if((distance >= 3.961)&&(distance <= 5.650))//5.726
		{
			kyori = 0;
		}
		else if((distance >= 5.651)&&(distance <= 6.300))
		{
			kyori = 2;
		}
		else if((distance >= 6.301)&&(distance <= 6.800))
		{
			kyori = 0;
		}
		else if((distance >= 6.801)&&(distance <= 7.554))
		{
			kyori = 2;
		}
		else if((distance >= 7.555)&&(distance <= 9.900))
		{
			kyori = 0;
		}
		else if((distance >= 9.901)&&(distance <= 10.400))
		{
			kyori = 4;
		}
		else if(distance >= 10.401)
		{
			kyori = 3;
		}
		else/* ①～⑦ */
		{
			kyori = -1;
		}

		switch(kyori)/* コースの特徴によって処理を変更 */
		{
			case 0 :/* 直線走行時 */

			/* 走行距離計算処理 */
			cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
			cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
			distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* タッチセンサで走行体ストップ */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			/* バランス走行用角度に制御 */
			tail_control(TAIL_ANGLE_DRIVE);

			forward_max = 150;/* unsigned char型 */

			/* PID制御 */
			diff[0] = diff[1];
			diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
			integral += (diff[1] + diff[0]) / 2.0 * 0.004;
			p = KP * diff[1];
			i = KI * integral;
			d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;/* KDを少なくし、走行を安定させる */
			turn = p + i + d;

			/* turnの上限を設定 */
			if( turn > 100 )
			{
				turn = 100;
			}
			else if( turn < -100 )
			{
				turn = -100;
			}

			/* 倒立振子制御API に渡すパラメーターを取得する */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

			/* 転倒検知 */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* 誤検知対策処理 */
		//		if(tentou > 50)
		//		{
		//			while(1)/* 完全停止処理 */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* 完全停止用カウントリセット */
		//	}

			/* バックラッシュキャンセル */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* 倒立振子制御APIを呼び出し、倒立走行するための */
			/* 左右モーター出力値を得る */
			balance_control(
				(float)forward_max,
				(float)turn,
				(float)gyro,
				(float)GYRO_OFFSET,
				(float)motor_ang_l,
				(float)motor_ang_r,
				(float)volt,
				(signed char*)&pwm_L,
				(signed char*)&pwm_R);

			/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
			/* 出力0時に、その都度設定する */
			if (pwm_L == 0)
			{
				ev3_motor_stop(left_motor, true);
			}
			else
			{
				ev3_motor_set_power(left_motor, (int)pwm_L);
			}
			if (pwm_R == 0)
			{
				ev3_motor_stop(right_motor, true);
			}
			else
			{
				ev3_motor_set_power(right_motor, (int)pwm_R);
			}

			tslp_tsk(4); /* 4msec周期起動 */

			break;

			case 1 : /* 緩カーブ走行時 */

			/* 走行距離計算処理 */
			cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
			cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
			distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* タッチセンサで走行体ストップ */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

			if (sonar_alert() == 1) /* 障害物検知 */
			{
				forward_max = turn = 0; /* 障害物を検知したら停止 */
			}
			else
			{
				forward_max = 150;

				/* PID制御 */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = KP * diff[1];
				i = KI * integral;
				d = (KD * 1.2) * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turnの上限を設定 */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* 倒立振子制御API に渡すパラメーターを取得する */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

			/* 転倒検知 */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* 誤検知対策処理 */
		//		if(tentou > 50)
		//		{
		//			while(1)/* 完全停止処理 */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* 完全停止用カウントリセット */
		//	}

			/* バックラッシュキャンセル */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* 倒立振子制御APIを呼び出し、倒立走行するための */
			/* 左右モーター出力値を得る */
			balance_control(
				(float)forward_max,
				(float)turn,
				(float)gyro,
				(float)GYRO_OFFSET,
				(float)motor_ang_l,
				(float)motor_ang_r,
				(float)volt,
				(signed char*)&pwm_L,
				(signed char*)&pwm_R);

			/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
			/* 出力0時に、その都度設定する */
			if (pwm_L == 0)
			{
				ev3_motor_stop(left_motor, true);
			}
			else
			{
				ev3_motor_set_power(left_motor, (int)pwm_L);
			}
			if (pwm_R == 0)
			{
				ev3_motor_stop(right_motor, true);
			}
			else
			{
				ev3_motor_set_power(right_motor, (int)pwm_R);
			}

			tslp_tsk(4); /* 4msec周期起動 */

			break;

			case 2 :/* 急カーブ走行時 */

			/* 走行距離計算処理 */
			cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
			cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
			distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* タッチセンサで走行体ストップ */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

			if (sonar_alert() == 1) /* 障害物検知 */
			{
				forward_max = turn = 0; /* 障害物を検知したら停止 */
			}
			else
			{
				forward_max = 150;/* unsigned char型 */

				/* PID制御 */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = (KP * 1.6) * diff[1];
				i = (KI * 0.9) * integral;
				d = KD * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turnの上限を設定 */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* 倒立振子制御API に渡すパラメーターを取得する */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* 転倒検知 */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* 誤検知対策処理 */
		//		if(tentou > 50)
		//		{
		//			while(1)/* 完全停止処理 */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* 完全停止用カウントリセット */
		//	}

			/* バックラッシュキャンセル */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* 倒立振子制御APIを呼び出し、倒立走行するための */
			/* 左右モーター出力値を得る */
			balance_control(
				(float)forward_max,
				(float)turn,
				(float)gyro,
				(float)GYRO_OFFSET,
				(float)motor_ang_l,
				(float)motor_ang_r,
				(float)volt,
				(signed char*)&pwm_L,
				(signed char*)&pwm_R);

				/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
				/* 出力0時に、その都度設定する */
			if (pwm_L == 0)
			{
				ev3_motor_stop(left_motor, true);
			}
			else
			{
				ev3_motor_set_power(left_motor, (int)pwm_L);
			}
			if (pwm_R == 0)
			{
				ev3_motor_stop(right_motor, true);
			}
			else
			{
				ev3_motor_set_power(right_motor, (int)pwm_R);
			}

			tslp_tsk(4); /* 4msec周期起動 */

			break;
			
			case 3 :
			while(1)/* ルックアップゲート */
			{
				/* 倒立振子制御APIに使用 */
				int32_t motor_ang_l, motor_ang_r;
				int volt,gyro;

				forward = 20;

				/* 走行距離計算処理 */
				cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
				cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
				distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
				distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
				distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
				distance += distance4ms;
				pre_angleL = cur_angleL;
				pre_angleR = cur_angleR;

				/* タッチセンサで走行体ストップ */
				if (ev3_button_is_pressed(BACK_BUTTON)) break;

				if (sonar_alert() == 1) /* 障害物検知 */
				{
					for(cnt=0;cnt<125;cnt++)/* ルックアップゲート準備 */
					{
						/* 走行体後退処理 */
						forward = -20;

						tail_control(LOOK_START_ANGLE);/* ルックアップ用角度に制御 */

						/* 走行距離計算処理 */
						cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
						cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
						distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						/* PID制御(閾値が白＋灰に変更) */
						diff[0] = diff[1];
						diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_GRAY) / 2));
						integral += (diff[1] + diff[0]) / 2.0 * 0.004;
						p = KP * diff[1];
						i = KI * integral;
						d = KD * (diff[1] - diff[0]) / 0.004;
						turn = p + i + d;

						/* 倒立振子制御API に渡すパラメーターを取得する */
						motor_ang_l = ev3_motor_get_counts(left_motor);
						motor_ang_r = ev3_motor_get_counts(right_motor);
						gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
						volt = ev3_battery_voltage_mV();

						/* 倒立振子制御APIを呼び出し、倒立走行するための */
						/* 左右モーター出力値を得る */
						balance_control(
							(float)forward,
							(float)turn,
							(float)gyro,
							(float)GYRO_OFFSET,
							(float)motor_ang_l,
							(float)motor_ang_r,
							(float)volt,
							(signed char*)&pwm_L,
							(signed char*)&pwm_R);

						/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
						/* 出力0時に、その都度設定する */
						if (pwm_L == 0)
						{
							ev3_motor_stop(left_motor, true);
						}
						else
						{
							ev3_motor_set_power(left_motor, (int)pwm_L);
						}
						if (pwm_R == 0)
						{
							ev3_motor_stop(right_motor, true);
						}
						else
						{
							ev3_motor_set_power(right_motor, (int)pwm_R);
						}

						tslp_tsk(4);
					}
					while(1)/* ルックアップゲート通過(壱回目) */
					{
						tail_control(LOOK_START_ANGLE);/* ルックアップ用角度に制御 */

						/* 走行体を前進させる */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* 走行距離計算処理 */
						cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
						cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
						distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);

						if(distance > 11.413)/* ルックアップゲート通過後 */
						{
							/* １秒間停止 */
							for(cnt=0;cnt<250;cnt++)
							{
								tail_control(LOOK_START_ANGLE);
								ev3_motor_stop(left_motor, false);
								ev3_motor_stop(right_motor, false);
								tslp_tsk(4);
							}
							break;
						}
					}

					while(1)/* ルックアップゲート通過(弐回目) */
					{
						tail_control(LOOK_START_ANGLE);/* ルックアップ用角度に制御 */

						/* 走行体を後退させる */
						ev3_motor_set_power(right_motor,LOOK_b_rpwm);
						ev3_motor_set_power(left_motor,LOOK_b_lpwm);

						/* 走行距離計算処理 */
						cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
						cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
						distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance < 10.753)/* ルックアップゲート通過後 */
						{
							/* １秒間停止 */
							for(cnt=0;cnt<250;cnt++)
							{
								tail_control(LOOK_START_ANGLE);
								ev3_motor_stop(left_motor, false);
								ev3_motor_stop(right_motor, false);
								tslp_tsk(4);
							}
							break;
						}
					}

					while(1)/* ルックアップゲート(参回目) */
					{
						tail_control(LOOK_START_ANGLE);/* ルックアップ用角度に制御 */

						/* 走行体を前進させる */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* 走行距離計算処理 */
						cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
						cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
						distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance > 11.413)/* ルックアップゲート通過後 */
						{
							/* １秒間停止 */
							for(cnt=0;cnt<250;cnt++)
							{
								tail_control(LOOK_START_ANGLE);
								ev3_motor_stop(left_motor, false);
								ev3_motor_stop(right_motor, false);
								tslp_tsk(4);
							}
							break;
						}
					}

					while(1)/* ガレージ */
					{

						tail_control(LOOK_START_ANGLE);/* ルックアップ用角度に制御 */

						/* 走行体を前進させる */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* 走行距離計算処理 */
						cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
						cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
						distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance > 11.863)/* ガレージ停止位置 */
						{
							while(1)/* おしまい */
							{
								tail_control(GARREZI);
								ev3_motor_stop(left_motor, false);
								ev3_motor_stop(right_motor, false);
								tslp_tsk(4);
							}
						}
					}
				}

				if (ev3_button_is_pressed(BACK_BUTTON)) break;

				/* タッチセンサで走行体ストップ */
				tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

				/* PID制御(閾値が白＋灰に変更) */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_GRAY) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = KP * diff[1];
				i = KI * integral;
				d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turnの上限を設定 */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}

				/* 倒立振子制御API に渡すパラメーターを取得する */
				motor_ang_l = ev3_motor_get_counts(left_motor);
				motor_ang_r = ev3_motor_get_counts(right_motor);
				gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
				volt = ev3_battery_voltage_mV();

		//		/* 転倒検知 */
		//		if(!(F_DOWN > gyro && gyro > B_DOWN))
		//		{
		//			tentou++;/* 誤検知対策処理 */
		//			if(tentou > 50)
		//			{
		//				while(1)/* 完全停止処理 */
		//				{
		//					ev3_motor_stop(left_motor, false);
		//					ev3_motor_stop(right_motor, false);
		//				}
		//			}
		//		}
		//		else
		//		{
		//			tentou = 0;/* 完全停止用カウントリセット */
		//		}

				/* バックラッシュキャンセル */
				backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

				/* 倒立振子制御APIを呼び出し、倒立走行するための */
				/* 左右モーター出力値を得る */
				balance_control(
					(float)forward,
					(float)turn,
					(float)gyro,
					(float)GYRO_OFFSET,
					(float)motor_ang_l,
					(float)motor_ang_r,
					(float)volt,
					(signed char*)&pwm_L,
					(signed char*)&pwm_R);

				/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
				/* 出力0時に、その都度設定する */
				if (pwm_L == 0)
				{
					ev3_motor_stop(left_motor, true);
				}
				else
				{
					ev3_motor_set_power(left_motor, (int)pwm_L);
				}
				if (pwm_R == 0)
				{
					ev3_motor_stop(right_motor, true);
				}
				else
				{
					ev3_motor_set_power(right_motor, (int)pwm_R);
				}

				tslp_tsk(4); /* 4msec周期起動 */
			}
			break;

			case 4:

			/* 走行距離計算処理 */
			cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
			cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
			distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* タッチセンサで走行体ストップ */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

			if (sonar_alert() == 1) /* 障害物検知 */
			{
				forward = turn = 0; /* 障害物を検知したら停止 */
			}
			else
			{
				forward = 70;/* unsigned char型 */

				/* PID制御 */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = (KP * 1.5) * diff[1];
				i = KI * integral;
				d = KD * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turnの上限を設定 */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* 倒立振子制御API に渡すパラメーターを取得する */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* 転倒検知 */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* 誤検知対策処理 */
		//		if(tentou > 50)
		//		{
		//			while(1)/* 完全停止処理 */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* 完全停止用カウントリセット */
		//	}

			/* バックラッシュキャンセル */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* 倒立振子制御APIを呼び出し、倒立走行するための */
			/* 左右モーター出力値を得る */
			balance_control(
				(float)forward,
				(float)turn,
				(float)gyro,
				(float)GYRO_OFFSET,
				(float)motor_ang_l,
				(float)motor_ang_r,
				(float)volt,
				(signed char*)&pwm_L,
				(signed char*)&pwm_R);

			/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
			/* 出力0時に、その都度設定する */
			if (pwm_L == 0)
			{
				ev3_motor_stop(left_motor, true);
			}
			else
			{
				ev3_motor_set_power(left_motor, (int)pwm_L);
			}
			if (pwm_R == 0)
			{
				ev3_motor_stop(right_motor, true);
			}
			else
			{
				ev3_motor_set_power(right_motor, (int)pwm_R);
			}

			tslp_tsk(4); /* 4msec周期起動 */

			break;

			default :/* 直線走行時 */

			/* 走行距離計算処理 */
			cur_angleL = ev3_motor_get_counts(left_motor);//左モータ回転角度の現在値
			cur_angleR = ev3_motor_get_counts(right_motor);//右モータ回転角度の現在値
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
			distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* タッチセンサで走行体ストップ */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			/* バランス走行用角度に制御 */
			tail_control(TAIL_ANGLE_DRIVE);

			forward_max = 150;/* unsigned char型 */

			/* PID制御 */
			diff[0] = diff[1];
			diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
			integral += (diff[1] + diff[0]) / 2.0 * 0.004;
			p = KP * diff[1];
			i = KI * integral;
			d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;/* KDを少なくし、走行を安定させる */
			turn = p + i + d;

			/* turnの上限を設定 */
			if( turn > 100 )
			{
				turn = 100;
			}
			else if( turn < -100 )
			{
				turn = -100;
			}

			/* 倒立振子制御API に渡すパラメーターを取得する */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* 転倒検知 */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* 誤検知対策処理 */
		//		if(tentou > 50)
		//		{
		//			while(1)/* 完全停止処理 */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* 完全停止用カウントリセット */
		//	}

			/* バックラッシュキャンセル */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* 倒立振子制御APIを呼び出し、倒立走行するための */
			/* 左右モーター出力値を得る */
			balance_control(
				(float)forward_max,
				(float)turn,
				(float)gyro,
				(float)GYRO_OFFSET,
				(float)motor_ang_l,
				(float)motor_ang_r,
				(float)volt,
				(signed char*)&pwm_L,
				(signed char*)&pwm_R);

			/* EV3ではモーター停止時のブレーキ設定が事前にできないため */
			/* 出力0時に、その都度設定する */
			if (pwm_L == 0)
			{
				ev3_motor_stop(left_motor, true);
			}
			else
			{
				ev3_motor_set_power(left_motor, (int)pwm_L);
			}
			if (pwm_R == 0)
			{
				ev3_motor_stop(right_motor, true);
			}
			else
			{
				ev3_motor_set_power(right_motor, (int)pwm_R);
			}

			tslp_tsk(4); /* 4msec周期起動 */

			break;
		}
	}
	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);

	ter_tsk(BT_TASK);
	fclose(bt);

	ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }
    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モーター目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モーターの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{

    /* PD制御を使って尻尾モーターを制御 */
    tail[0] = tail[1];
    tail[1] = angle - ev3_motor_get_counts(tail_motor);
    signed int pwm = (signed int)tail[1]*P_GAIN + (tail[1]-tail[0])*D_GAIN/0.004; /* 比例制御 */

  //signed int pwm = (signed int)((angle - ev3_motor_get_counts(tail_motor))*P_GAIN); /* 比例制御 */

    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }
    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
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
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}

//*****************************************************************************
// 関数名 : backlash_cancel
// 引数 : lpwm (左モーターPWM値 ※前回の出力値)
//        rpwm (右モーターPWM値 ※前回の出力値)
//        lenc (左モーターエンコーダー値)
//        renc (右モーターエンコーダー値)
// 返り値 : なし
// 概要 : 直近のPWM値に応じてエンコーダー値にバックラッシュ分の値を追加します。
//*****************************************************************************
void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc)
{
    const int BACKLASHHALF = 4;   // バックラッシュの半分[deg]

    if(lpwm < 0) *lenc += BACKLASHHALF;
    else if(lpwm > 0) *lenc -= BACKLASHHALF;

    if(rpwm < 0) *renc += BACKLASHHALF;
    else if(rpwm > 0) *renc -= BACKLASHHALF;
}

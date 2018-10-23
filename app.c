/**
 ******************************************************************************
 ** �t�@�C���� : app.c
 **
 ** �T�v : L�R�[�X���s�v���O����
 **
 ** �l�@ : �E�����̒l���s����
 **        �EPID����̒l���s����
 **        �Eswitch���ɂ��v���O�����̒x���̉\���L��
 **        �E�ꕔ�v���O�����̊ȗ������\
 **        �E���P�b�g�X�^�[�g�̒������K�v
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
 * �Z���T�[�A���[�^�[�̐ڑ����`���܂�
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

static int      bt_cmd = 0;     /* Bluetooth�R�}���h 1:�����[�g�X�^�[�g */
static FILE     *bt = NULL;     /* Bluetooth�t�@�C���n���h�� */

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */

/* sample_c1�}�N�� */
#define GYRO_OFFSET  0          /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define LIGHT_WHITE  40         /* ���F�̌��Z���T�l */
#define LIGHT_BLACK  1          /* ���F�̌��Z���T�l */
#define LIGHT_GRAY  20          /* �D�F�̌��Z���T�l */

/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] */

/* sample_c3�}�N�� */
#define TAIL_ANGLE_STAND_UP  85 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F /* ���S��~�p���[�^�[������W�� */
#define D_GAIN             0.028F /* ���S��~�p���[�^�[���� */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^�[����PWM��΍ő�l */

static float tail[2] = {0,0}; /* ���S��~�p���[�^�[���� */

/* sample_c4�}�N�� */
//#define DEVICE_NAME     "ET150"  /* Bluetooth�� sdcard:\ev3rt\etc\rc.conf.ini LocalName�Őݒ� */
//#define PASS_KEY        "9753" /* �p�X�L�[    sdcard:\ev3rt\etc\rc.conf.ini PinCode�Őݒ� */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* PID���� */
#define KP 0.42
#define KI 0.06
#define KD 0.060

/* ���P�b�g�X�^�[�g */
#define TAIL_ANGLE_ROCKET_START 88

/* �]�|臒l */
#define F_DOWN 30
#define B_DOWN -30

/* ���s�����v�Z */
#define TIRE_DIAMETER 0.098
#define PI 3.14

/* �֐��v���g�^�C�v�錾 */
static int sonar_alert(void);
static void tail_control(signed int angle);
static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);

/* ���b�N�A�b�v�Q�[�g */
#define stop 0
#define LOOK_START_ANGLE 63
#define LOOK_rpwm 20
#define LOOK_lpwm 20
#define LOOK_b_rpwm -20
#define LOOK_b_lpwm -20

#define GARREZI 75

/* EV3�V�X�e������ */
static user_system_create(){

}

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
	signed char forward;      /* �O��i���� */
	unsigned char forward_max; /* foward���� */
	signed char turn;         /* ���񖽗� */
	signed char pwm_L, pwm_R; /* ���E���[�^�[PWM�o�� */

	/* PID���� */
	float p;
	float i;
	float d;
	float diff[2] = {0,0};
	float integral = 0;

	/* ���s�����v�Z���� */
	float distance4msL = 0;
	float distance4msR = 0;
	float distance4ms = 0;
	double distance = 0.0;
	float cur_angleL = 0;
	float cur_angleR = 0;
	float pre_angleL = 0;
	float pre_angleR = 0;

	/* �p�^�[������ */
	int kyori;

	/* �]�|���m */
	int tentou = 0;

	/* ���P�b�g�X�^�[�g */
	int dash;

	/* ���b�N�A�b�v�Q�[�g */
	int cnt = 0;/* ���b�N�A�b�v�Q�[�g���̑S�Ă�for���Ŏg�p */

	/* LCD��ʕ\�� */
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	ev3_lcd_draw_string("EV3way-ET practice-1", 0, CALIB_FONT_HEIGHT*1);

	/* �Z���T�[���̓|�[�g�̐ݒ� */
	ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
	ev3_sensor_config(color_sensor, COLOR_SENSOR);
	ev3_color_sensor_get_reflect(color_sensor); /* ���˗����[�h */
	ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
	ev3_sensor_config(gyro_sensor, GYRO_SENSOR);

	/* ���[�^�[�o�̓|�[�g�̐ݒ� */
	ev3_motor_config(left_motor, LARGE_MOTOR);
	ev3_motor_config(right_motor, LARGE_MOTOR);
	ev3_motor_config(tail_motor, LARGE_MOTOR);
	ev3_motor_reset_counts(tail_motor);

	/* Open Bluetooth file */
	bt = ev3_serial_open_file(EV3_SERIAL_BT);
	assert(bt != NULL);

	/* Bluetooth�ʐM�^�X�N�̋N�� */
	act_tsk(BT_TASK);

	ev3_led_set_color(LED_ORANGE); /* �����������ʒm */

	/* �X�^�[�g�ҋ@ */

	tslp_tsk(1000);

	while(1)
	{
		tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */

		if (bt_cmd == 1)
		{
			break; /* �����[�g�X�^�[�g */
		}

		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
		{
			break; /* �^�b�`�Z���T�������ꂽ */
		}

		tslp_tsk(10); /* 10msec�E�F�C�g */
	}

	/* �W���C���Z���T�[���Z�b�g */
	ev3_gyro_sensor_reset(gyro_sensor);
	balance_init(); /* �|���U�qAPI������ */

	/* ���s���[�^�[�G���R�[�_�[���Z�b�g */
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);

	ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */

	for(dash=0;dash<5;dash++)
	{
		tail_control(TAIL_ANGLE_ROCKET_START); /* ���P�b�g�X�^�[�g */
		tslp_tsk(4); /* 4msec�E�F�C�g */
	}

	/*
	* Main loop for the self-balance control algorithm
	*/

	/* ���s�X�^�[�g */
	while(1)
	{

		/* while�����łP�x��` */
		/* �|���U�q����API�Ɏg�p */
		int32_t motor_ang_l, motor_ang_r;
		int gyro, volt;

		/* �p�^�[������ */
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
		else/* �@�`�F */
		{
			kyori = -1;
		}

		switch(kyori)/* �R�[�X�̓����ɂ���ď�����ύX */
		{
			case 0 :/* �������s�� */

			/* ���s�����v�Z���� */
			cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
			cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
			distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			/* �o�����X���s�p�p�x�ɐ��� */
			tail_control(TAIL_ANGLE_DRIVE);

			forward_max = 150;/* unsigned char�^ */

			/* PID���� */
			diff[0] = diff[1];
			diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
			integral += (diff[1] + diff[0]) / 2.0 * 0.004;
			p = KP * diff[1];
			i = KI * integral;
			d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;/* KD�����Ȃ����A���s�����肳���� */
			turn = p + i + d;

			/* turn�̏����ݒ� */
			if( turn > 100 )
			{
				turn = 100;
			}
			else if( turn < -100 )
			{
				turn = -100;
			}

			/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

			/* �]�|���m */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* �댟�m�΍􏈗� */
		//		if(tentou > 50)
		//		{
		//			while(1)/* ���S��~���� */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//	}

			/* �o�b�N���b�V���L�����Z�� */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
			/* ���E���[�^�[�o�͒l�𓾂� */
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

			/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
			/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

			tslp_tsk(4); /* 4msec�����N�� */

			break;

			case 1 : /* �ɃJ�[�u���s�� */

			/* ���s�����v�Z���� */
			cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
			cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
			distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

			if (sonar_alert() == 1) /* ��Q�����m */
			{
				forward_max = turn = 0; /* ��Q�������m�������~ */
			}
			else
			{
				forward_max = 150;

				/* PID���� */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = KP * diff[1];
				i = KI * integral;
				d = (KD * 1.2) * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turn�̏����ݒ� */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

			/* �]�|���m */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* �댟�m�΍􏈗� */
		//		if(tentou > 50)
		//		{
		//			while(1)/* ���S��~���� */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//	}

			/* �o�b�N���b�V���L�����Z�� */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
			/* ���E���[�^�[�o�͒l�𓾂� */
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

			/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
			/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

			tslp_tsk(4); /* 4msec�����N�� */

			break;

			case 2 :/* �}�J�[�u���s�� */

			/* ���s�����v�Z���� */
			cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
			cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
			distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

			if (sonar_alert() == 1) /* ��Q�����m */
			{
				forward_max = turn = 0; /* ��Q�������m�������~ */
			}
			else
			{
				forward_max = 150;/* unsigned char�^ */

				/* PID���� */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = (KP * 1.6) * diff[1];
				i = (KI * 0.9) * integral;
				d = KD * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turn�̏����ݒ� */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* �]�|���m */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* �댟�m�΍􏈗� */
		//		if(tentou > 50)
		//		{
		//			while(1)/* ���S��~���� */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//	}

			/* �o�b�N���b�V���L�����Z�� */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
			/* ���E���[�^�[�o�͒l�𓾂� */
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

				/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
				/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

			tslp_tsk(4); /* 4msec�����N�� */

			break;
			
			case 3 :
			while(1)/* ���b�N�A�b�v�Q�[�g */
			{
				/* �|���U�q����API�Ɏg�p */
				int32_t motor_ang_l, motor_ang_r;
				int volt,gyro;

				forward = 20;

				/* ���s�����v�Z���� */
				cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
				cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
				distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
				distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
				distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
				distance += distance4ms;
				pre_angleL = cur_angleL;
				pre_angleR = cur_angleR;

				/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
				if (ev3_button_is_pressed(BACK_BUTTON)) break;

				if (sonar_alert() == 1) /* ��Q�����m */
				{
					for(cnt=0;cnt<125;cnt++)/* ���b�N�A�b�v�Q�[�g���� */
					{
						/* ���s�̌�ޏ��� */
						forward = -20;

						tail_control(LOOK_START_ANGLE);/* ���b�N�A�b�v�p�p�x�ɐ��� */

						/* ���s�����v�Z���� */
						cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
						cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
						distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						/* PID����(臒l�����{�D�ɕύX) */
						diff[0] = diff[1];
						diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_GRAY) / 2));
						integral += (diff[1] + diff[0]) / 2.0 * 0.004;
						p = KP * diff[1];
						i = KI * integral;
						d = KD * (diff[1] - diff[0]) / 0.004;
						turn = p + i + d;

						/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
						motor_ang_l = ev3_motor_get_counts(left_motor);
						motor_ang_r = ev3_motor_get_counts(right_motor);
						gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
						volt = ev3_battery_voltage_mV();

						/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
						/* ���E���[�^�[�o�͒l�𓾂� */
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

						/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
						/* �o��0���ɁA���̓s�x�ݒ肷�� */
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
					while(1)/* ���b�N�A�b�v�Q�[�g�ʉ�(����) */
					{
						tail_control(LOOK_START_ANGLE);/* ���b�N�A�b�v�p�p�x�ɐ��� */

						/* ���s�̂�O�i������ */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* ���s�����v�Z���� */
						cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
						cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
						distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);

						if(distance > 11.413)/* ���b�N�A�b�v�Q�[�g�ʉߌ� */
						{
							/* �P�b�Ԓ�~ */
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

					while(1)/* ���b�N�A�b�v�Q�[�g�ʉ�(����) */
					{
						tail_control(LOOK_START_ANGLE);/* ���b�N�A�b�v�p�p�x�ɐ��� */

						/* ���s�̂���ނ����� */
						ev3_motor_set_power(right_motor,LOOK_b_rpwm);
						ev3_motor_set_power(left_motor,LOOK_b_lpwm);

						/* ���s�����v�Z���� */
						cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
						cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
						distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance < 10.753)/* ���b�N�A�b�v�Q�[�g�ʉߌ� */
						{
							/* �P�b�Ԓ�~ */
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

					while(1)/* ���b�N�A�b�v�Q�[�g(�Q���) */
					{
						tail_control(LOOK_START_ANGLE);/* ���b�N�A�b�v�p�p�x�ɐ��� */

						/* ���s�̂�O�i������ */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* ���s�����v�Z���� */
						cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
						cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
						distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance > 11.413)/* ���b�N�A�b�v�Q�[�g�ʉߌ� */
						{
							/* �P�b�Ԓ�~ */
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

					while(1)/* �K���[�W */
					{

						tail_control(LOOK_START_ANGLE);/* ���b�N�A�b�v�p�p�x�ɐ��� */

						/* ���s�̂�O�i������ */
						ev3_motor_set_power(right_motor,LOOK_rpwm);
						ev3_motor_set_power(left_motor,LOOK_lpwm);

						/* ���s�����v�Z���� */
						cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
						cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
						distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
						distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
						distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
						distance += distance4ms;
						pre_angleL = cur_angleL;
						pre_angleR = cur_angleR;

						tslp_tsk(4);
						if(distance > 11.863)/* �K���[�W��~�ʒu */
						{
							while(1)/* �����܂� */
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

				/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
				tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

				/* PID����(臒l�����{�D�ɕύX) */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_GRAY) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = KP * diff[1];
				i = KI * integral;
				d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turn�̏����ݒ� */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}

				/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
				motor_ang_l = ev3_motor_get_counts(left_motor);
				motor_ang_r = ev3_motor_get_counts(right_motor);
				gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
				volt = ev3_battery_voltage_mV();

		//		/* �]�|���m */
		//		if(!(F_DOWN > gyro && gyro > B_DOWN))
		//		{
		//			tentou++;/* �댟�m�΍􏈗� */
		//			if(tentou > 50)
		//			{
		//				while(1)/* ���S��~���� */
		//				{
		//					ev3_motor_stop(left_motor, false);
		//					ev3_motor_stop(right_motor, false);
		//				}
		//			}
		//		}
		//		else
		//		{
		//			tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//		}

				/* �o�b�N���b�V���L�����Z�� */
				backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

				/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
				/* ���E���[�^�[�o�͒l�𓾂� */
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

				/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
				/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

				tslp_tsk(4); /* 4msec�����N�� */
			}
			break;

			case 4:

			/* ���s�����v�Z���� */
			cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
			cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
			distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

			if (sonar_alert() == 1) /* ��Q�����m */
			{
				forward = turn = 0; /* ��Q�������m�������~ */
			}
			else
			{
				forward = 70;/* unsigned char�^ */

				/* PID���� */
				diff[0] = diff[1];
				diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
				integral += (diff[1] + diff[0]) / 2.0 * 0.004;
				p = (KP * 1.5) * diff[1];
				i = KI * integral;
				d = KD * (diff[1] - diff[0]) / 0.004;
				turn = p + i + d;

				/* turn�̏����ݒ� */
				if( turn > 100 )
				{
					turn = 100;
				}
				else if( turn < -100 )
				{
					turn = -100;
				}
			}

			/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* �]�|���m */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* �댟�m�΍􏈗� */
		//		if(tentou > 50)
		//		{
		//			while(1)/* ���S��~���� */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//	}

			/* �o�b�N���b�V���L�����Z�� */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
			/* ���E���[�^�[�o�͒l�𓾂� */
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

			/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
			/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

			tslp_tsk(4); /* 4msec�����N�� */

			break;

			default :/* �������s�� */

			/* ���s�����v�Z���� */
			cur_angleL = ev3_motor_get_counts(left_motor);//�����[�^��]�p�x�̌��ݒl
			cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
			distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
			distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
			distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
			distance += distance4ms;
			pre_angleL = cur_angleL;
			pre_angleR = cur_angleR;

			/* �^�b�`�Z���T�ő��s�̃X�g�b�v */
			if (ev3_button_is_pressed(BACK_BUTTON)) break;

			/* �o�����X���s�p�p�x�ɐ��� */
			tail_control(TAIL_ANGLE_DRIVE);

			forward_max = 150;/* unsigned char�^ */

			/* PID���� */
			diff[0] = diff[1];
			diff[1] = (ev3_color_sensor_get_reflect(color_sensor) - ((LIGHT_WHITE + LIGHT_BLACK) / 2));
			integral += (diff[1] + diff[0]) / 2.0 * 0.004;
			p = KP * diff[1];
			i = KI * integral;
			d = (KD * 0.5) * (diff[1] - diff[0]) / 0.004;/* KD�����Ȃ����A���s�����肳���� */
			turn = p + i + d;

			/* turn�̏����ݒ� */
			if( turn > 100 )
			{
				turn = 100;
			}
			else if( turn < -100 )
			{
				turn = -100;
			}

			/* �|���U�q����API �ɓn���p�����[�^�[���擾���� */
			motor_ang_l = ev3_motor_get_counts(left_motor);
			motor_ang_r = ev3_motor_get_counts(right_motor);
			gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
			volt = ev3_battery_voltage_mV();

		//	/* �]�|���m */
		//	if(!(F_DOWN > gyro && gyro > B_DOWN))
		//	{
		//		tentou++;/* �댟�m�΍􏈗� */
		//		if(tentou > 50)
		//		{
		//			while(1)/* ���S��~���� */
		//			{
		//				ev3_motor_stop(left_motor, false);
		//				ev3_motor_stop(right_motor, false);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		tentou = 0;/* ���S��~�p�J�E���g���Z�b�g */
		//	}

			/* �o�b�N���b�V���L�����Z�� */
			backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

			/* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
			/* ���E���[�^�[�o�͒l�𓾂� */
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

			/* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
			/* �o��0���ɁA���̓s�x�ݒ肷�� */
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

			tslp_tsk(4); /* 4msec�����N�� */

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
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
    {
        /*
         * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
         * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
         * EV3�̏ꍇ�́A�v�m�F
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* ��Q�������m */
        }
        else
        {
            alert = 0; /* ��Q������ */
        }
        counter = 0;
    }
    return alert;
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�[�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�[�̊p�x����
//*****************************************************************************
static void tail_control(signed int angle)
{

    /* PD������g���ĐK�����[�^�[�𐧌� */
    tail[0] = tail[1];
    tail[1] = angle - ev3_motor_get_counts(tail_motor);
    signed int pwm = (signed int)tail[1]*P_GAIN + (tail[1]-tail[0])*D_GAIN/0.004; /* ��ᐧ�� */

  //signed int pwm = (signed int)((angle - ev3_motor_get_counts(tail_motor))*P_GAIN); /* ��ᐧ�� */

    /* PWM�o�͖O�a���� */
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
// �֐��� : bt_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ��M */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* �G�R�[�o�b�N */
    }
}

//*****************************************************************************
// �֐��� : backlash_cancel
// ���� : lpwm (�����[�^�[PWM�l ���O��̏o�͒l)
//        rpwm (�E���[�^�[PWM�l ���O��̏o�͒l)
//        lenc (�����[�^�[�G���R�[�_�[�l)
//        renc (�E���[�^�[�G���R�[�_�[�l)
// �Ԃ�l : �Ȃ�
// �T�v : ���߂�PWM�l�ɉ����ăG���R�[�_�[�l�Ƀo�b�N���b�V�����̒l��ǉ����܂��B
//*****************************************************************************
void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc)
{
    const int BACKLASHHALF = 4;   // �o�b�N���b�V���̔���[deg]

    if(lpwm < 0) *lenc += BACKLASHHALF;
    else if(lpwm > 0) *lenc -= BACKLASHHALF;

    if(rpwm < 0) *renc += BACKLASHHALF;
    else if(rpwm > 0) *renc -= BACKLASHHALF;
}

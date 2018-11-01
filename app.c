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
#include "strategy/walker/BalanceWalker.h"
#include "strategy/walker/TailWalker.h"
#include "app/Calibration.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//���
typedef enum State {
	STATE_INIT = 0,		//�������
	STATE_CAL,			//�L�����u���[�V������
	STATE_READY,		//�ҋ@��
	STATE_RUNNING,		//���s��
	STATE_STOP,			//���S��~��
	STATE_FAIL,			//�]�|��
	STATE_SAVE,			//���s���O�ۑ���
	STATE_END,			//�I�����
	STATE_TEST,			//������
	TNUM_STATE			//��Ԃ̐�(��ԍŌ�̍s�ɏ�������)
} State;

//�C�x���g
typedef enum Event {
	EVENT_NONE = 0,			//�C�x���g������
	EVENT_TOUCH_IS_PRESSED,	//�^�b�`�Z���T�[��������
	EVENT_BT_START,			//�����[�g�X�^�[�g����
	EVENT_BT_STOP,			//�����[�g�X�g�b�v����
	EVENT_FAIL,				//�]�|����
	TNUM_EVENT,				//�C�x���g�̐��i��ԍŌ�̍s�ɏ������Ɓj
} Event;

//�I�u�W�F�N�g��ÓI�Ɋm�ۂ���
TouchSensor	gTouchSensor;		//�^�b�`�Z���T
SonarSensor gSonarSensor;		//�����g�Z���T
ColorSensor gColorSensor;		//�J���[�Z���T�[�i���ˌ����[�h�j
GyroSensor  gGyroSensor;		//�W���C���Z���T
Motor 		gLeftMotor;			//�����[�^
Motor 		gRightMotor;		//�E���[�^
Motor 		gTailMotor;			//�K�����[�^

/* EV3�V�X�e������ */
static void user_system_create(){

	//�f�o�C�X������
	TouchSensor_init(&gTouchSensor, EV3_PORT_1);
	SonarSensor_init(&gSonarSensor, EV3_PORT_2);
	ColorSensor_init(&gColorSensor, EV3_PORT_3);
	GyroSensor_init( &gGyroSensor,  EV3_PORT_4);
	Motor_init(&gLeftMotor,  EV3_PORT_C, LARGE_MOTOR);
	Motor_init(&gRightMotor, EV3_PORT_B, LARGE_MOTOR);
	Motor_init(&gTailMotor,  EV3_PORT_A, LARGE_MOTOR);

	//�I�u�W�F�N�g�̏������ƃ����N�t��


	//�����������ʒm
	ev3_led_set_color(LED_ORANGE);
}

//�I������
static void user_system_destroy(){
	Motor_stop(&gLeftMotor);
	Motor_stop(&gRightMotor);
	Motor_stop(&gTailMotor);
}

//�C�x���g�m�F
static Event checkEvent(){

	Event e = EVENT_NONE;

	//�C�x���g�������m�F���āA�G�x���g����Ԃ�
	if(TouchSensor_isPressed(&gTouchSensor)) e = EVENT_TOUCH_IS_PRESSED;
	else e = EVENT_NONE;

	return e;
}

//�C�x���g�n���h��
static State handleEvent(State s, Event e){

	State ss = s;	//���݂̏�Ԃ�ۑ�

	switch(ss){
		case STATE_INIT:	//�������
			user_system_create();	//����������
			ss = STATE_TEST;		//��ԑJ��(�������->������)
			break;

		case STATE_CAL:		//�L�����u���[�V������
			break;
		case STATE_READY:	//�ҋ@��
			break;
		case STATE_RUNNING:	//���s��
			break;
		case STATE_STOP:	//���S��~��
			break;
		case STATE_FAIL:	//�]�|��
			break;
		case STATE_SAVE:	//���s���O�ۑ���
			break;
		case STATE_END:		//�I�����
			user_system_destroy();	//�I������
			break;

		case STATE_TEST:	//������
			if(e == EVENT_TOUCH_IS_PRESSED) {
				ev3_speaker_play_tone(NOTE_C4, 10);
			}

			if(ev3_button_is_pressed(UP_BUTTON)) ss = STATE_END;

			break;

		default:			//�G���[����
			break;
		}

	return ss;	//���̏�Ԃ�Ԃ�
}

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
	State state = STATE_INIT;	//��ԕϐ���������Ԃɐݒ�
	Event event = EVENT_NONE;	//�C�x���g�ϐ����C�x���g�������ɐݒ�
	
	while(1){
		event = checkEvent();				//�C�x���g�������m�F�A�C�x���g����Ԃ�
		state = handleEvent(state, event);	//���݂̏�ԂƃC�x���g����ɂ��ď������s�A���ɏ�ԑJ�ڂ���
		tslp_tsk(4);	//4ms�ҋ@
	}
	ext_tsk();

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
 
}


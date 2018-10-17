/*
 * Sound.h
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#ifndef ROBOT_DEVICE_SOUND_H_
#define ROBOT_DEVICE_SOUND_H_

#include "ev3api.h"

typedef struct Sound {
	uint16_t mFrequency;
	int32_t mDuration;
	uint8_t mVolume;
} Sound;

void Sound_init(Sound* this, uint16_t freq, int32_t dur, uint8_t vol);
void Sound_setVolume(Sound* this, uint8_t vol);
void Sound_playTone(Sound* this);

#endif /* ROBOT_DEVICE_SOUND_H_ */

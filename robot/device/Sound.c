/*
 * Sound.c
 *
 *  Created on: 2018/10/17
 *      Author: hashi
 */

#include "Sound.h"

void Sound_init(Sound* this, uint16_t freq, int32_t dur, uint8_t vol){
    this->mFrequency = freq;
    this->mDuration = dur;
    this->mVolume = vol;
    Sound_setVolume(this, this->mVolume);
}

void Sound_setVolume(Sound* this, uint8_t vol){
    ev3_speaker_set_volume(this->mVolume);
}

void Sound_playTone(Sound* this){
    ev3_speaker_play_tone(this->mFrequency, this->mDuration);
}

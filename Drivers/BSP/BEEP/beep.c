/*
 * beep.c
 *
 *  Created on: May 30, 2025
 *      Author: 1
 */


#include "beep.h"

#define LED_NUM 3
#define SAMPLE_SIZE 480 * 20
#define LED_MESSAGE_NUM 8
#define SYNC_CODE_NUM 8
#define LED_CODE_NUM 8
#define FFH_CODE_NUM 8
#define TONE_MAX_NUM   100
#define SYNC_NUM_MAX 15 * 20

#define MUSIC_START	0xF0	//乐谱开始标志
#define MUSIC_END	0xFF	//乐谱结束标志

extern uint8_t logic_buffer[SAMPLE_SIZE];
extern int sync_index[SYNC_NUM_MAX];
extern uint8_t sync_num;
extern int led_index[SYNC_NUM_MAX];

extern float tones[TONE_MAX_NUM];	//音调, 范围为tone_frequency中的14个音阶
extern uint8_t beats[TONE_MAX_NUM];	//节拍, 两只老虎乐谱中最短为1/4拍为250ms, 将其定为1, 1/2拍为2, 1拍为4
extern uint8_t tone_index;
extern uint8_t is_recording;

const float tone_frequency[14] = {
		//低音
		//1      2        3        4        5        6     7
		261.63,  293.67,  329.63,  349.23,  391.99,  440,  493.88,
		//中音
		//1      2        3        4        5        6     7
		532.25,  587.33,  659.25,  698.46,  783.99,  880,  987.76
};

void get_music() {
	for(int i = 0; i < sync_num - 5; i++) {
		int led = led_index[i];
		int index = sync_index[i];
		if(led < 0 || led >= LED_NUM) continue;

		uint8_t led_message = 0;
		for(int j = 0; j < LED_MESSAGE_NUM; j++) {
			uint8_t logic = logic_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM + FFH_CODE_NUM];
			led_message |= (logic << (LED_MESSAGE_NUM - 1 - j));	//提取8位bit
		}

		if(led_message == MUSIC_START) {
			is_recording = 1;
			tone_index = 0;
		}
		else if(led_message == MUSIC_END) {
			is_recording = 0;
		}
		else if(is_recording) {
			if(tone_index < TONE_MAX_NUM) {
				//提取音调和节拍信息
				float tone = tone_frequency[ (led_message >> 4) & 0x0F ];	//高4位音调信息
				uint8_t beat = led_message & 0x0F;		//低4位节拍信息
				if(tone_index < TONE_MAX_NUM) {
					tones[tone_index] = tone;
					beats[tone_index] = beat;
					tone_index++;
				}
			}
		}
	}
}

void play_music() {
	uint32_t i, delay_time, tone;

	for(i = 0; i < tone_index; i++) {
		delay_time = beats[i] * 250;

		tone = (uint32_t)84*1000*1000 / tones[i];
		//TIM4->ARR = tone;	//改变频率
		__HAL_TIM_SET_AUTORELOAD(&htim4, tone);
//		TIM4->CCR1 = tone / 2;	//占空比为50%
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tone / 2);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_Delay(delay_time);	//节拍延时
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);	//CNT寄存器清0
	}
}

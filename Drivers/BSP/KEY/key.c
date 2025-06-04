/*
 * key.c
 *
 *  Created on: Jun 4, 2025
 *      Author: 1
 */

#include "key.h"


uint8_t key_scan() {
	uint8_t key_val = HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin);

	switch(key_state) {
	case KEY_IDLE:
		if(key_val == KEY_PRESSED) {
			key_state = KEY_DEBOUNCE;
			debounce_count = 0;
		}
		break;
	case KEY_DEBOUNCE:
		if(key_val == KEY_PRESSED) {
			if(debounce_count >= DEBOUNCE_TIME) {
				key_state = KEY_PRESSED_OK;
				key_event = 1;	//检测到按键按下
				HAL_TIM_Base_Stop_IT(&htim5);
			}
			else {
				HAL_TIM_Base_Start_IT(&htim5);
			}
		}
		else {
			//抖动或松开
			key_state = KEY_IDLE;
		}
		break;
	case KEY_PRESSED_OK:
		if(key_val == KEY_RELEASE) {
			key_state = KEY_IDLE;
		}
		break;
	default:
		key_state = KEY_IDLE;
		break;
	}
	if(key_event) {
		//有按键按下事件
		key_event = 0;
		return 1;
	}
	//没有按键按下
	return 0;
}

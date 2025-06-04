/*
 * key.h
 *
 *  Created on: Jun 4, 2025
 *      Author: 1
 */

#ifndef BSP_KEY_KEY_H_
#define BSP_KEY_KEY_H_

#include "main.h"
#include "tim.h"

#define DEBOUNCE_TIME	10	//消抖延时ms
#define KEY_PRESSED	1	//按键按下
#define KEY_RELEASE 0	//按键释放

typedef enum {
	KEY_IDLE,
	KEY_DEBOUNCE,
	KEY_PRESSED_OK
}KeyState_t;

static KeyState_t key_state = KEY_IDLE;
extern uint8_t debounce_count;
static uint8_t key_event = 0;	//置1表示检测到一次按下事件

uint8_t key_scan();

#endif /* BSP_KEY_KEY_H_ */

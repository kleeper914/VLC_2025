/*
 * beep.h
 *
 *  Created on: May 30, 2025
 *      Author: 1
 *
 *  参考文章: https://blog.csdn.net/qq_44800056/article/details/139129319
 */

#ifndef BSP_BEEP_BEEP_H_
#define BSP_BEEP_BEEP_H_

#include <stdint.h>
#include "main.h"
#include "tim.h"

/*
const float pitch_frequency[21] = {
		//低音
		//1      2        3        4        5        6     7
		261.63,  293.67,  329.63,  349.23,  391.99,  440,  493.88,
		//中音
		//1      2        3        4        5        6     7
		532.25,  587.33,  659.25,  698.46,  783.99,  880,  987.76,
		//高音
		//1      2        3        4        5        6     7
		1046.50, 1174.66, 1318.51, 1396.62, 1567.98, 1760, 1975.52
};
*/


void get_music();
void play_music();

#endif /* BSP_BEEP_BEEP_H_ */

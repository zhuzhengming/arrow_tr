/*
 * Mission_Launch.h
 *
 *  Created on: 2021年5月17日
 *      Author: shengsifan
 */

#ifndef INC_M3508_CTRL_H_
#define INC_M3508_CTRL_H_
#include "wtr_definition.h"
#include "gpio.h"
#include "math.h"

#include <stdio.h>
#include "DJI.h"
#include "main.h" //解决ITM_SendChar()未定义的错误；直接包含core_cm7.h，会很多错误。

#define MAX_ACC		200	// 复位滑块最大加速度, mm/(s*s)
#define MAX_VEL 	  	200	// 复位滑块最大速度, mm/s
#define TOUR_WHEN_TRIANGLE_FAST		(MAX_VEL*MAX_VEL/MAX_ACC)	// 三角形和梯形速度规划的分水岭运动距离（快）
#define MMS2RPM (60*3591.0f / 187.0f/ (PI * 40))

void moveToTargetPoint(float target_point, int motorId);

#endif /* INC_M3508_CTRL_H_ */

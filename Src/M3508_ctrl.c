/*
 * Mission_Launch.c
 *
 *  Created on: 2021年5月17日
 *      Author: shengsifan
 */
#include "M3508_ctrl.h"



int Kc =30;

typedef struct {
	float vel;
	float pos;
} MotorTarget_t;

/**
 * @brief 梯形速度规划
 * @param startPos: 起始位置
 *          endPos: 目标位置
 *           cur_t: 当前时间点
 *           max_v: 最大速度
 *           max_a: 最大加速度
 *   *motor_target: 复位滑块状态
 */
void trapezoidPlanning(float startPos, float endPos, float cur_t, float max_v,
		float max_a, MotorTarget_t *motor_target) {

	float tour = endPos - startPos;			// 路程，末减初
	float Pos_Ought = 0, Speed_Ought = 0; // 每个时刻复位滑块理论上应该的位移和速度

	if (fabs(tour) <= max_v*max_v/max_a) {					// 三角形规划
		float timeAll = 2 * sqrt(fabs(tour) / max_a);		// 用的总时间
		if (cur_t > timeAll * 1000) {
			cur_t = timeAll * 1000;
		}
		if (cur_t < timeAll * 500) {					// 前一半时间
			Speed_Ought = max_a * (cur_t) / 1000;
			Pos_Ought = 0.5 * max_a * ((cur_t) / 1000) * ((cur_t) / 1000);
		} else {
			Speed_Ought = max_a * (timeAll - (cur_t) / 1000);
			Pos_Ought = fabs(tour)
					- 0.5 * max_a * (timeAll - (cur_t) / 1000)
							* (timeAll - (cur_t) / 1000);
		}
	}
	else {

		float timeAll = max_v / max_a + fabs(tour) / max_v;
		if (cur_t > timeAll * 1000) {
			cur_t = timeAll * 1000;
		}

		if (cur_t < 1000 * max_v / max_a) {
			Speed_Ought = max_a * (cur_t) / 1000;
			Pos_Ought = 0.5 * max_a * ((cur_t) / 1000) * ((cur_t) / 1000);
		} else if (cur_t >= 1000 * max_v / max_a
				&& cur_t < 1000 * (timeAll - max_v / max_a)) {
			Speed_Ought = max_v;
			Pos_Ought = max_v * max_v * 0.5 / max_a
					+ max_v * ((cur_t) / 1000 - max_v / max_a);
		} else {
			Speed_Ought = max_a * (timeAll - (cur_t) / 1000);
			Pos_Ought = fabs(tour)
					- 0.5 * max_a * (timeAll - (cur_t) / 1000)
							* (timeAll - (cur_t) / 1000);
		}

	}
	motor_target->vel = Speed_Ought;
	motor_target->pos = Pos_Ought;
}


/**
 * @brief 复位滑块移动到指定位置
 * @param target_point: 目标位置
 *             motorId: 电机ID
 */
MotorTarget_t cur_tar[3];
void moveToTargetPoint(float target_point, int motorId) {
	static float last_target[3] = {-1,-1,-1};
	static uint32_t startTime[3] = {0,0,0};
	static float startPos[3];
	if (fabs(last_target[motorId] - target_point) > 0.01) {
		startTime[motorId] = uwTick;
		startPos[motorId] = hDJI[motorId].Pos_on_Track;	// 函数开始时所在点为起始点
	}
	float cur_t = (uwTick - startTime[motorId]);
	// 开始计时
	int SIG = (target_point - startPos[motorId]) > 0 ? 1 : -1;

	trapezoidPlanning(startPos[motorId], target_point, cur_t, MAX_VEL, MAX_ACC,
			&(cur_tar[motorId]));
	float posError = startPos[motorId] + cur_tar[motorId].pos * SIG - hDJI[motorId].Pos_on_Track;
	hDJI[motorId].speedPID.ref = MMS2RPM * (SIG * cur_tar[motorId].vel + Kc * posError);
	last_target[motorId] = target_point;
}

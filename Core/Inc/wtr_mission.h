#ifndef WTR_MISSION_H
#define WTR_MISSION_H

#include "wtr_definition.h"

void GM6020_ctrl();
void M3508_ctrl();
void ElectroMag_ctrl();
void Laser_ctrl();//张佳辉提供 读出来的激光数据直接存到sendbag里面
void Feedback_ctrl();
void Led_ctrl();

#endif
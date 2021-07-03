#ifndef GM6020_H
#define GM6020_H

#include "can.h"
#include "DJI.h"

typedef struct
{

    struct{
        float   RotorAngle_0_360;						// 电机转子角度 单位 度° 范围0~360
        float   rpm;									// 电机转子速度，单位 rpm
        float   current;								// 电机转矩电流
    }FdbData;

    struct{
        float     rpm_calc;							    // 机械转子角度差分出的rpm
        float     last_RotorAngle_0_360;                // 上次反馈的角度 单位度 0~360
        int       last_tick;                            // 上一次差分速度的时间
    }Calculate;

    float encoder_resolution;							// 编码器分辨率

    PID_t speedPID;
    PID_t posPID;
}GM6020_t;

extern GM6020_t h6020s[7];

void CanTransmit_GM6020_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CanTransmit_GM6020_567(CAN_HandleTypeDef *hcanx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq);
void GM6020_Init();
void GM6020_Update(int id, uint8_t *fdbData);
HAL_StatusTypeDef GM6020_CanMsgDecode(uint32_t stdid,uint8_t *fdbData);
#endif

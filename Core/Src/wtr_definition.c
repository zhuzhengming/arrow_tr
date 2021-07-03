#include "wtr_definition.h"

const uint8_t header[2] = {0xff, 0x86};

RecvMsg_t recvMsg;
Bag_t sendBag;

uint8_t JoyStickReceiveData[18];
uint8_t UART6ReceiveData[BAG_LENGTH*2];
uint8_t CanReceiveData[8];


//void PID_Calc(PID_t *pid)
//{
//	pid->cur_error = pid->ref - pid->fdb;
//	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
//	pid->error[0] = pid->error[1];
//	pid->error[1] = pid->ref - pid->fdb;
//	/*设定输出上限*/
//	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
//	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
//
//}

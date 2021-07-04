#include "wtr_uart.h"

#include "string.h"
#include "math.h"
#include "usart.h"

#define RAWDATA_DEADZONE 250

Remote_t Raw_Data;


void UART1Decode()
{
    Raw_Data.ch0 = ((int16_t)JoyStickReceiveData[0] | ((int16_t)JoyStickReceiveData[1] << 8)) & 0x07FF;
    Raw_Data.ch1 = (((int16_t)JoyStickReceiveData[1] >> 3) | ((int16_t)JoyStickReceiveData[2] << 5)) & 0x07FF;
    Raw_Data.ch2 = (((int16_t)JoyStickReceiveData[2] >> 6) | ((int16_t)JoyStickReceiveData[3] << 2) |
                                                ((int16_t)JoyStickReceiveData[4] << 10)) & 0x07FF;
    Raw_Data.ch3 = (((int16_t)JoyStickReceiveData[4] >> 1) | ((int16_t)JoyStickReceiveData[5]<<7)) & 0x07FF;
    Raw_Data.left = ((JoyStickReceiveData[5] >> 4) & 0x000C) >> 2;
    Raw_Data.right = ((JoyStickReceiveData[5] >> 4) & 0x0003);

    // if(fabs(Raw_Data.ch0-CH1_BIAS) < RAWDATA_DEADZONE ){
    //     Raw_Data.ch0 =CH0_BIAS;
    // }
    // if(fabs(Raw_Data.ch1-CH1_BIAS) < RAWDATA_DEADZONE ){
    //     Raw_Data.ch1 =CH1_BIAS;
    // }
    // if(fabs(Raw_Data.ch2-CH1_BIAS) < RAWDATA_DEADZONE ){
    //     Raw_Data.ch2 =CH2_BIAS;
    // }
    // if(fabs(Raw_Data.ch3-CH1_BIAS) < RAWDATA_DEADZONE ){
    //     Raw_Data.ch3 =CH3_BIAS;
    // }

    switch(Raw_Data.left)
    {
        case 1:

            /* left choice 1 */
            break;
        case 3:

            /* left choice 2 */
            break;
        case 2:

            /* left choice 3 */
            break;
        default:
            break;
    }

    switch(Raw_Data.right)
    {
        case 1:

            /* right choice 1 */
            break;
        case 3:

            /* right choice 2 */
            break;
        case 2:

            /* right choice 3 */
            break;
        default:
            break;
    }
}

#define MAX_MOTORPOS 450

void UART6Decode(){
    for (int i = 0; i < BAG_LENGTH; i++) {
		Bag_t tempBag = *(Bag_t*) (void*) (&(UART6ReceiveData[i]));
		if (tempBag.head[0] == header[0] &&
		    tempBag.head[1] == header[1]) {

            recvMsg = *(RecvMsg_t*)(void*)(tempBag.payload);

            if(recvMsg.left.motorPos<0)	recvMsg.left.motorPos = 0;
            if(recvMsg.mid.motorPos<0)	recvMsg.mid.motorPos = 0;
            if(recvMsg.right.motorPos<0)	recvMsg.right.motorPos = 0;
            if(recvMsg.yaw<0)	recvMsg.yaw = 0;

            if(recvMsg.left.motorPos>MAX_MOTORPOS)	recvMsg.left.motorPos = MAX_MOTORPOS;
            if(recvMsg.mid.motorPos>MAX_MOTORPOS)	recvMsg.mid.motorPos = MAX_MOTORPOS;
            if(recvMsg.right.motorPos>MAX_MOTORPOS)	recvMsg.right.motorPos = MAX_MOTORPOS;
            if(recvMsg.yaw>PI)	recvMsg.yaw = PI;

			break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
    if(huart == &huart1){
        UART1Decode();
    }
    if(huart->Instance == huart6.Instance){
        UART6Decode();
        HAL_UART_Receive_DMA(&huart6, UART6ReceiveData, BAG_LENGTH*2 );

    }

}

#include "wtr_mission.h"
#include "math.h"
#include "M3508_ctrl.h"
#define K 100

void GM6020_ctrl(){

    h6020s[0].posPID.fdb = h6020s[0].FdbData.RotorAngle_0_360;
    h6020s[0].posPID.ref = recvMsg.yaw;

    h6020s[0].posPID.cur_error = h6020s[0].posPID.ref - h6020s[0].posPID.fdb;
    h6020s[0].posPID.output = h6020s[0].posPID.KP * h6020s[0].posPID.cur_error;
    if( fabs(h6020s[0].posPID.output) > h6020s[0].posPID.outputMax ) h6020s[0].posPID.output = _SIGN(h6020s[0].posPID.output) * h6020s[0].posPID.outputMax;

    h6020s[0].speedPID.ref = h6020s[0].posPID.output;
	PID_Calc(&h6020s[0].speedPID);
	PID_Calc(&h6020s[1].speedPID);

	CanTransmit_GM6020_1234(&hcan1, h6020s[0].speedPID.output, 0,0,0);
}
void M3508_ctrl(){

	/*recvMsg.left.motorPos = K*recvMsg.left.laserDistance;
	 * recvMsg.mid.motorPos = K*recvMsg.mid.laserDistance;
	 * recvMsg.right.motorPos = K*recvMsg.rignt.laserDistance;
	 *
	 */
    moveToTargetPoint(recvMsg.left.motorPos, 0);
    moveToTargetPoint(recvMsg.mid.motorPos, 1);
    moveToTargetPoint(recvMsg.right.motorPos, 2);

	PID_Calc(&hDJI[0].speedPID);
	PID_Calc(&hDJI[1].speedPID);
	PID_Calc(&hDJI[2].speedPID);
    CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output, hDJI[1].speedPID.output, hDJI[2].speedPID.output, 0);
}
void ElectroMag_ctrl(){
    if(recvMsg.left.electromagnetOrder == 0x02){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    }

    if(recvMsg.mid.electromagnetOrder == 0x02){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    }

    if(recvMsg.right.electromagnetOrder == 0x02){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
    }
}


#define MAX_DST_10V 10000.0//10m
#define MIN_DST_0V 100.0//10cm
 
uint16_t spi_data[8]={0};
uint8_t *pData=(uint8_t *)spi_data;
float sick_dst[8]={0};

void Laser_ctrl(){

    if(!HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin))
    {
		HAL_GPIO_WritePin(CAB_GPIO_Port, CAB_Pin, GPIO_PIN_RESET);//CAB, negative sample postive hold
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//CSL
		HAL_SPI_Receive(&hspi4, pData, 8,100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//CSH
		HAL_GPIO_WritePin(CAB_GPIO_Port, CAB_Pin, GPIO_PIN_SET);//CAB
      for(int i=7;i+1;i--)sick_dst[i]=spi_data[i]/32768.0f*(MAX_DST_10V-MIN_DST_0V)+MIN_DST_0V;
    }

    recvMsg.left.laserDistance = sick_dst[0];
    recvMsg.mid.laserDistance = sick_dst[1];
    recvMsg.right.laserDistance = sick_dst[2];
}

void Feedback_ctrl(){
    HAL_UART_Transmit_IT(&huart6, (uint8_t*)sendBag.raw, BAG_LENGTH );
    HAL_UART_Transmit_IT(&huart6, (uint8_t*)sendBag.raw, BAG_LENGTH );
}

void Led_ctrl(){
    static int cnt = 50;
    if((--cnt)<=0){
          cnt = 10;
          HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
      }
}

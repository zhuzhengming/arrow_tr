#include "wtr_mission.h"
#include "math.h"
#include "M3508_ctrl.h"

void GM6020_ctrl(){
    //speed
    h6020s[0].posPID.fdb = h6020s[0].FdbData.RotorAngle_0_360;
    h6020s[0].posPID.ref = recvMsg.yaw;

    h6020s[0].posPID.cur_error = h6020s[0].posPID.ref - h6020s[0].posPID.fdb;
    h6020s[0].posPID.output = h6020s[0].posPID.KP * h6020s[0].posPID.cur_error;
    if( fabs(h6020s[0].posPID.output) > h6020s[0].posPID.outputMax ) h6020s[0].posPID.output = _SIGN(h6020s[0].posPID.output) * h6020s[0].posPID.outputMax;

    h6020s[0].speedPID.ref = h6020s[0].posPID.output;
	PID_Calc(&h6020s[0].speedPID);
	PID_Calc(&h6020s[1].speedPID);

	CanTransmit_GM6020_1234(&hcan1, h6020s[0].speedPID.output, 0,0,0);


    /*	position

if(recvMsg.yaw>80)	h6020s[0].posPID.ref = 180;
else	h6020s[0].posPID.ref = 0;
h6020s[0].posPID.fdb = h6020s[0].AxisData.AxisAngle_inDegree/2.5;
h6020s[0].speedPID.output = h6020s[0].posPID.KP*(h6020s[0].posPID.ref-h6020s[0].posPID.fdb);

if(fabs(h6020s[0].speedPID.output) > h6020s[0].posPID.outputMax){
	if(h6020s[0].speedPID.output>0)	h6020s[0].speedPID.ref = h6020s[0].posPID.outputMax;
	else
	{	
		h6020s[0].speedPID.ref = -h6020s[0].posPID.outputMax;
	}
	
}

h6020s[0].speedPID.fdb = h6020s[0].FdbData.rpm;

PID_Calc(&h6020s[0].speedPID);
CanTransmit_GM6020_1234(&hcan1, h6020s[0].speedPID.output, 0,0,0);
}

*/


void M3508_ctrl(){

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
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    if(recvMsg.mid.electromagnetOrder == 0x02){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    }

    if(recvMsg.right.electromagnetOrder == 0x02){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
    }
}


#define MAX_DST_10V 3000.0
#define MIN_DST_0V 50.0

uint16_t spi_data[8]={0};
uint8_t *pData=(uint8_t *)spi_data;
float sick_dst[8]={0};

void Laser_ctrl(){

    while(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0));

    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);//CSL
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);//CAB
    HAL_SPI_Receive(&hspi4, pData, 8,100);
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);//CSH
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);//CAB
    for(int i=7;i+1;i--){
        sick_dst[i]=spi_data[i]/32768.0*(MAX_DST_10V-MIN_DST_0V)+MIN_DST_0V;//读取出的距离 单位mm
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

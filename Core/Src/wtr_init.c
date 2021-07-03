#include "wtr_init.h"
#include "wtr_can.h"

void laser_init(){
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);//CAB, negative sample
    for(uint8_t t=0xF; t; t--);
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);//CAB

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}


void WTR_Init()
{
    
    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18);
    HAL_UART_Receive_DMA(&huart6, UART6ReceiveData, BAG_LENGTH*2);
    CANFilterInit(&hcan1);
    HAL_TIM_Base_Start_IT(&htim1);
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;
    DJI_Init();

    GM6020_Init();

    sendBag.head[0] = header[0];
    sendBag.head[1] = header[1];

    laser_init();
}

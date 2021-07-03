#include "GM6020.h"
#include "math.h"

GM6020_t h6020s[7];
void GM6020_Init() 
{
	for(int i =0; i<7; i++)
	{
			h6020s[i].speedPID.KP = 150;
			h6020s[i].speedPID.KI = 10;
			h6020s[i].speedPID.KD = 1;
			h6020s[i].speedPID.outputMax = 30000;

			h6020s[i].posPID.KP = 40.0f;
			h6020s[i].posPID.outputMax = 60;

			h6020s[i].encoder_resolution = 8192.0f;

	}
}

static uint32_t TxMailbox;

void CanTransmit_GM6020_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq){
	CAN_TxHeaderTypeDef TxMessage;
		
	TxMessage.DLC=0x08;
	TxMessage.StdId=0x1FF;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;

	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)cm1_iq;
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)cm2_iq;
	TxData[4] = (uint8_t)(cm3_iq >> 8);
	TxData[5] = (uint8_t)cm3_iq;
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)cm4_iq;
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0) ;
	if(HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,&TxMailbox)!=HAL_OK)
	{
		 Error_Handler();       //如果CAN信息发送失败则进入死循环
	}
}

void CanTransmit_GM6020_567(CAN_HandleTypeDef *hcanx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq) {
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC = 0x08;
    TxMessage.StdId = 0x2FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;

    uint8_t TxData[8];
    TxData[0] = (uint8_t) (cm5_iq >> 8);
    TxData[1] = (uint8_t) cm5_iq;
    TxData[2] = (uint8_t) (cm6_iq >> 8);
    TxData[3] = (uint8_t) cm6_iq;
    TxData[4] = (uint8_t) (cm7_iq >> 8);
    TxData[5] = (uint8_t) cm7_iq;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
    if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();       //如果CAN信息发送失败则进入死循环
    }
}
HAL_StatusTypeDef GM6020_CanMsgDecode(uint32_t stdid,uint8_t *fdbData) 
{
    int id =stdid-0x204;
    if( id>=0 && id<7 ){
        GM6020_Update(id, fdbData);
        return HAL_OK;
    }

    return HAL_ERROR;
}

void GM6020_Update(int id, uint8_t *fdbData) {
	static int last_tick[7];
	static int ave_cnt[7];
	/*  反馈信息计算  */
	h6020s[id].FdbData.RotorAngle_0_360 = (fdbData[0] << 8 | fdbData[1]) * 360.0f
			/ h6020s[id].encoder_resolution; /* unit:degree*/
    h6020s[id].FdbData.rpm = (fdbData[2] << 8 | fdbData[3]);
    h6020s[id].FdbData.current = (fdbData[4] << 8 | fdbData[5]);


	if(ave_cnt[id]--==0){
		ave_cnt[id]=5;
		h6020s[id].Calculate.rpm_calc = (h6020s[id].FdbData.RotorAngle_0_360 -h6020s[id].Calculate.last_RotorAngle_0_360)*
		                            (1.0f/(uwTick-h6020s[id].Calculate.last_tick)*1000.0*60.0f/360.0f);
        if(isnan(h6020s[id].Calculate.rpm_calc)) h6020s[id].Calculate.rpm_calc = h6020s[id].FdbData.rpm;
        h6020s[id].Calculate.last_RotorAngle_0_360 = h6020s[id].FdbData.RotorAngle_0_360;
		h6020s[id].Calculate.last_tick=uwTick;
    }
}




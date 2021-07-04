#include "wtr_definition.h"

#include "M3508_ctrl.h"
#include "math.h"
#include "wtr_mission.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if( htim->Instance == htim1.Instance ){
        //GM6020_ctrl();
        M3508_ctrl();
        ElectroMag_ctrl();
    }
}

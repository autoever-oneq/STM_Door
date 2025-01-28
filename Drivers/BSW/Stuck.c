#include "stm32l0xx_hal.h"
#include "Stuck.h"

extern TIM_HandleTypeDef htim3;			// Front stuck timer
extern TIM_HandleTypeDef htim6;			// Back stuck timer

uint8_t reset_flag[2];							// To prevent fast start...

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)		// 300ms Timer callback
{
	if(htim == &htim3)			// Front
	{
		if(reset_flag[0])
			HAL_GPIO_WritePin(Stuck_GPIO_Port, Stuck_Result_Front_Pin, GPIO_PIN_SET);	// Front-door stuck occur
		else
			reset_flag[0] = 1;
	}
	if(htim == &htim6)			// Back
	{
		if(reset_flag[1])
			HAL_GPIO_WritePin(Stuck_GPIO_Port, Stuck_Result_Back_Pin, GPIO_PIN_SET);	// Back-door stuck occur
		else
			reset_flag[1] = 1;
	}
}
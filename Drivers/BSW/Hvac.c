#include "stm32l0xx_hal.h"
#include "Hvac.h"

// HVAC on/off
void Hvac()
{
	HVAC_GPIO_Port->ODR ^= HVAC_Output_Pin;
	
	return;
}
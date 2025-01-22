#include "stm32l0xx_hal.h"
#include "DoorIo.h"

// Change LED depending on lock state
void LockToLED(uint16_t Lock_Pin, door_lock_t state)
{
	GPIO_PinState PinState = GPIO_PIN_RESET;
	
	// Pin & Output mapping
	PinState = (state == Lock ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
	// HAL_GPIO
	HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, PinState);
}
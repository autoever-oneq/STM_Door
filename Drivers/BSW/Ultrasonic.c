#include "stm32l0xx_hal.h"
#include "Ultrasonic.h"

#define LD2_GPIO_Port		GPIOA
#define LD2_Pin					GPIO_PIN_5

/* Extern variables in main.c */
extern TIM_HandleTypeDef htim7;
extern volatile uint8_t flag_done[2];
extern volatile double dist[2];
/* Extern variables in main.c */

// ULTRASONIC FUNC
void UltraSonic()
{
	// First 10cm check
	SignalTrig();
	
	// Send obstacle detection results to the Master STM board
	uint32_t result_front, result_back;
	
	result_front = (dist[Front] <= 10 ? Result_Front_Pin : 0);
	result_back = (dist[Back] <= 10 ? Result_Back_Pin : 0);
	Result_GPIO_Port->ODR &= ~(Result_Front_Pin | Result_Back_Pin);
	Result_GPIO_Port->ODR |= (result_front | result_back);
	
	return;
}

// Ultrasonic trigger signal
void SignalTrig()
{
	// Reset done flags
	flag_done[Front] = flag_done[Back] = 0;
	
	// Front & Back trigger signal
	Trig_GPIO_Port->ODR |= (Trig_Front_Pin | Trig_Back_Pin);
	usdelay(10);		// 10us delay
	Trig_GPIO_Port->ODR &= ~(Trig_Front_Pin | Trig_Back_Pin);
	
	// Wait until echo signal received
	while(flag_done[Front] == 0 || flag_done[Back] == 0);
}

// Polling version usdelay(TIM7)
void usdelay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim7, 0);							// Counter initialize to 0
	while(__HAL_TIM_GET_COUNTER(&htim7) < time);	// time(us) delay
}
#include "stm32l0xx_hal.h"
#include "Ultrasonic.h"

#define LD2_GPIO_Port		GPIOA
#define LD2_Pin					GPIO_PIN_5

/* Extern variable in main.c */
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern volatile double dist;
extern volatile char flag_100ms;
/* Extern variable in main.c */

// Counter tick
volatile int cnt_100ms;

// ULTRASONIC FUNC
void Ultrasonic()
{
	int cnt = 0;
	
	// First 10cm check
	SignalTrig();
	if(dist < 10)					// Obstacle detected
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		HAL_Delay(100);
		return;							// Door state not changed (x,0) -> (x,0)
	}
	
	// Obstacle not detected : door state change (x,0) -> (1,1)
	HAL_TIM_Base_Start_IT(&htim6);				// Start 100ms timer
	while(cnt < 15)												// Req time for fully open (1.5s)
	{
		if(flag_100ms == 1)									// Trig signal period : TIM6(100ms)
		{
			flag_100ms = 0;										// Flag reset
			SignalTrig();											// Send trigger signal
			
			if(dist >= 10)										// Safety distance (distance >= 10)
			{
				cnt += !cnt_100ms;							// if(cnt_100ms == 0) cnt++;
				cnt_100ms -= (cnt_100ms > 0);		// if(cnt_100ms > 0) cnt_100ms--;
			}
			else
			{
				cnt_100ms = 10;									// Wait (1s) until obstacle is gone
			}
		}
	}
	
	// Door fully opened : Stop 100ms counter
	HAL_TIM_Base_Stop_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	
	// (Transmit 'DoorStateChanged')
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	HAL_Delay(100);
	
	return;
}

// Ultrasonic trigger signal
void SignalTrig()
{
	// Trigger signal
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
	usdelay(10);		// 10us delay
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
	
	// Wait for echo signal ((4m*2) / 340m/s = 25ms (include margin))
	HAL_Delay(25);
}

// Polling version usdelay(TIM7)
void usdelay(unsigned short time)
{
	__HAL_TIM_SET_COUNTER(&htim7, 0);							// Counter initialize to 0
	while(__HAL_TIM_GET_COUNTER(&htim7) < time);	// time(us) delay
}
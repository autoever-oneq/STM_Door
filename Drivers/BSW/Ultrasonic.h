#ifndef __ULTRASONIC__H
#define __ULTRASONIC__H

#include <stdint.h>

#define Trig_GPIO_Port								GPIOB
#define Trig_Front_Pin								GPIO_PIN_5		// (CN9) D4
#define Trig_Back_Pin									GPIO_PIN_10		// (CN9) D6

#define Echo_GPIO_Port								GPIOB
#define Echo_Front_Pin								GPIO_PIN_4		// (CN9) D5
#define Echo_Back_Pin									GPIO_PIN_6		// (CN9) D10

#define Ultrasonic_Result_GPIO_Port		GPIOA
#define Ultrasonic_Result_Front_Pin		GPIO_PIN_7		// (CN9) D11
#define Ultrasonic_Result_Back_Pin		GPIO_PIN_6		// (CN9) D12

typedef enum Seat
{
	Front,
	Back
}seat_t;

extern void UltraSonic();
extern void SignalTrig();
extern void usdelay(uint16_t time);

#endif
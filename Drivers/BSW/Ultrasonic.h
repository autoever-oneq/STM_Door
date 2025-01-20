#ifndef __ULTRASONIC__H
#define __ULTRASONIC__H

#define Trig_GPIO_Port	GPIOB
#define Trig_Pin				GPIO_PIN_5
#define Echo_GPIO_Port	GPIOB
#define Echo_Pin				GPIO_PIN_4

extern void Ultrasonic();
extern void SignalTrig();
extern void usdelay(unsigned short time);

#endif
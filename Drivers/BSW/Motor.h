#ifndef __MOTOR__
#define __MOTOR__
#include "main.h"

typedef enum Rotation_t{
	CLOCKWISE = 68,
	STOP = 74,
	COUNTERCLOCKWISE = 83
}Rotation;

typedef enum {
    MOTOR_STOP,
    MOTOR_ROTATING_CLOCKWISE,
    MOTOR_ROTATING_COUNTERCLOCKWISE
}MotorState;

typedef struct MotorInfo_t{
	TIM_HandleTypeDef *sttim;
	uint32_t TimerChannel;
	uint8_t TargetAngle;
	uint8_t CurrentAngle;
	MotorState state;
	uint8_t RotationFactor;
}MotorInfo;


void MotorInit(MotorInfo *ServoMotor, TIM_HandleTypeDef *sttim,uint32_t TimerChannel);
uint8_t MotorSetRotationFactor(MotorInfo *ServoMotor);
void MotorSetTargetAngle(MotorInfo *ServoMotor, uint16_t Angle);
void MotorSetRotation(MotorInfo *ServoMotor,uint8_t Time);
void MotorStop(MotorInfo *ServoMotor);
void MotorActuate(MotorInfo *ServoMotor);

/*
void MotorInit(TIM_HandleTypeDef *sttim,uint32_t TimerChannel);
void MotorSetTargetAngle(uint16_t Angle);
void MotorSetRotation(uint8_t Time);
void MotorStop();
void MotorActuate();
*/
extern MotorInfo Motor1;
extern MotorInfo Motor2;

#endif
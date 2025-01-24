#ifndef __MOTOR__
#define __MOTOR__
#include "main.h"


typedef enum {
    MOTOR_STOP,
    MOTOR_OPEN,
    MOTOR_CLOSE
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
//uint8_t MotorSetRotationFactor(MotorInfo *ServoMotor);
void MotorSetTargetAngle(MotorInfo *ServoMotor, uint8_t Angle);
void MotorSetRotation(MotorInfo *ServoMotor);
void MotorStop(MotorInfo *ServoMotor);
void MotorActuate(MotorInfo *ServoMotor);

extern MotorInfo Motor1;
extern MotorInfo Motor2;

#endif
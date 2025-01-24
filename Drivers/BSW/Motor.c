#include "Motor.h"
#include "Door.h"
#include "UART.h"

MotorInfo Motor1;
MotorInfo Motor2;

extern UART_HandleTypeDef huart1;
//Motor1 12~180 degree actuation 

void MotorInit(MotorInfo *ServoMotor, TIM_HandleTypeDef *sttim, uint32_t TimerChannel){
	ServoMotor->sttim = sttim;
	ServoMotor->TimerChannel = TimerChannel;
	ServoMotor->state = MOTOR_STOP;
	ServoMotor->RotationFactor = 10;
	ServoMotor->TargetAngle = (150 * (150 / ServoMotor->RotationFactor));
	ServoMotor->CurrentAngle = (150 * (150 / ServoMotor->RotationFactor));
	HAL_TIM_PWM_Start(ServoMotor->sttim,ServoMotor->TimerChannel); //PWM Start
}

void MotorSetTargetAngle(MotorInfo *ServoMotor, uint8_t Angle) {
	ServoMotor->TargetAngle = (Angle * (Angle / ServoMotor->RotationFactor)) ;
}

void MotorStop(MotorInfo *ServoMotor){
	/*
	if (ServoMotor->state == MOTOR_OPEN){
		uart1_tx_data[0] = ;
	}
	*/
	/*
	if (ServoMotor->state == MOTOR_CLOSE){
		
	}
	*/
	ServoMotor->state = MOTOR_STOP;
	ServoMotor->TargetAngle = ServoMotor->CurrentAngle;
	MotorSetRotation(ServoMotor);
}

void MotorActuate(MotorInfo *ServoMotor){
	if (ServoMotor -> TargetAngle > ServoMotor -> CurrentAngle){		
		ServoMotor -> CurrentAngle = ServoMotor-> CurrentAngle + ServoMotor->RotationFactor;	
		MotorSetRotation(ServoMotor);	
		ServoMotor->state = MOTOR_OPEN;
	}
	if (ServoMotor->TargetAngle < ServoMotor->CurrentAngle){
		ServoMotor->CurrentAngle = ServoMotor-> CurrentAngle - ServoMotor->RotationFactor;
		MotorSetRotation(ServoMotor);	
		ServoMotor->state = MOTOR_CLOSE;
	}
	if (ServoMotor->TargetAngle == ServoMotor->CurrentAngle){
		MotorStop(ServoMotor);
		ServoMotor->state = MOTOR_STOP;
	}
}


void MotorSetRotation(MotorInfo *ServoMotor){
	uint8_t ActuateAngle = (uint8_t)(( 0.589 * ServoMotor->CurrentAngle ) + 18.11);
	__HAL_TIM_SetCompare(ServoMotor->sttim,ServoMotor->TimerChannel, ActuateAngle);
}

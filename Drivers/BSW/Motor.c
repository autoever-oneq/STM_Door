#include "main.h"
#include "Motor.h"



MotorInfo Motor1;
MotorInfo Motor2;

void MotorInit(MotorInfo *ServoMotor, TIM_HandleTypeDef *sttim, uint32_t TimerChannel){
	ServoMotor->sttim = sttim;
	ServoMotor->TimerChannel = TimerChannel;
	ServoMotor->CurrentAngle = 90;//default degree
	ServoMotor->TargetAngle = 90;//default degree
	ServoMotor->state = MOTOR_STOP;
	ServoMotor->RotationFactor = MotorSetRotationFactor(ServoMotor);
	HAL_TIM_PWM_Start(ServoMotor->sttim,ServoMotor->TimerChannel); //PWM Start
}

uint8_t MotorSetRotationFactor(MotorInfo *ServoMotor){
	if (ServoMotor == &Motor1){
		return 5;
	}
	else if (ServoMotor == &Motor2){
		return 5;
	}
	else return -1;
	
}

void MotorSetTargetAngle(MotorInfo *ServoMotor, uint16_t Angle) {
	uint16_t Target;
	if (Angle <= 0){
		Target = 0;
	}
	else if (Angle >= 180){
		Target = 180;
	}
	else Target = Angle - (Angle % ServoMotor->RotationFactor);
	//ServoMotor->TargetAngle = Angle;
	ServoMotor->TargetAngle = Target;
}

void MotorStop(MotorInfo *ServoMotor){
	ServoMotor->TargetAngle = ServoMotor->CurrentAngle;
	MotorSetRotation(ServoMotor, STOP);
}

void MotorActuate(MotorInfo *ServoMotor){
	if (ServoMotor->TargetAngle > ServoMotor->CurrentAngle){
		ServoMotor->CurrentAngle = ServoMotor-> CurrentAngle + ServoMotor->RotationFactor;
		MotorSetRotation(ServoMotor, CLOCKWISE);
	}
	else if (ServoMotor->TargetAngle < ServoMotor->CurrentAngle){
		ServoMotor->CurrentAngle = ServoMotor->CurrentAngle - ServoMotor->RotationFactor;
		MotorSetRotation(ServoMotor, COUNTERCLOCKWISE);
	}
	else if (ServoMotor->TargetAngle == ServoMotor->CurrentAngle){
		MotorStop(ServoMotor);
	}	
}

void MotorSetRotation(MotorInfo *ServoMotor, uint8_t Time){
	__HAL_TIM_SetCompare(ServoMotor->sttim,ServoMotor->TimerChannel,Time);
}

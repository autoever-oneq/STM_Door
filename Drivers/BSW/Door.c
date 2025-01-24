#include "main.h"
#include "Door.h"
#include "Motor.h"
#include "UART.h"

DoorInfo Door1;
DoorInfo Door2;

extern MotorInfo Motor1; 
extern MotorInfo Motor2; 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t uart1_tx_data[4];

void DoorInit(DoorInfo *Door, MotorInfo *ServoMotor) {
	Door->ServoMotor = ServoMotor;
	if (Door->ServoMotor == &Motor1){
		Door->DoorPos = FRONT;
		Door->UltraPort = GPIOC;
		Door->UltraPin = GPIO_PIN_2;
	}
	if (Door->ServoMotor == &Motor2){
		Door->DoorPos = REAR;
		Door->UltraPort = GPIOC;
		Door->UltraPin = GPIO_PIN_3;
	}
	Door->LockFlag = 1;
	Door->UnlockFlag = 0;
	Door->OpenFlag = 0;
	Door->CloseFlag = 1;
	Door->IsObstacle = 0;
}

void DoorOpen(DoorInfo *Door){
	MotorSetTargetAngle(Door->ServoMotor , 90);
}

void DoorClose(DoorInfo *Door){
	MotorSetTargetAngle(Door->ServoMotor , 150);
}

void DoorActuate(DoorInfo *Door){
	
	if (Door->LockFlag) {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);	
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);	
		return;
	}
	if (Door->UnlockFlag) {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);	
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1);	
		Door->IsObstacle = HAL_GPIO_ReadPin(Door->UltraPort,Door->UltraPin);
		if (!Door->IsObstacle){
			if(Door->OpenFlag){
				DoorOpen(Door);
				MotorActuate(Door->ServoMotor);		
			}
		}
		if(Door->CloseFlag){
			DoorClose(Door);
			MotorActuate(Door->ServoMotor);
		}
	}
}

void DoorLockCommand(void){
	if((Door1.LockFlag == 1)&& (Door2.LockFlag == 1)) return;
	if ((Door1.CloseFlag == 1) && (Door2.CloseFlag == 1)){
			Door1.LockFlag = 1;
			Door1.UnlockFlag = 0;
			Door2.LockFlag = 1;
			Door2.UnlockFlag = 0;	
	}
	UartTx(0x20,0x03, 0x00);
}

void DoorUnlockCommand(void){
	if((Door1.UnlockFlag == 1)&& (Door2.UnlockFlag == 1)) return;
	Door1.LockFlag = 0;
	Door1.UnlockFlag = 1;
	Door2.LockFlag = 0;
	Door2.UnlockFlag = 1;
	UartTx(0x20,0x03, 0x10);
}

void DoorCloseCommand(DoorInfo *Door) {
	if(Door->CloseFlag == 1) return;
	if (Door->UnlockFlag){
				Door->OpenFlag = 0;
				Door->CloseFlag = 1;	
	}
	if (Door == &Door1){
		UartTx(0x20,0x01, 0x10);
	}
	if (Door == &Door2){
		UartTx(0x20,0x02, 0x10);
	}
}

void DoorOpenCommand(DoorInfo *Door) {
	if(Door->OpenFlag == 1) return;
	if (Door->UnlockFlag){
		Door->OpenFlag = 1;
		Door->CloseFlag = 0;	
	}	
	if (Door == &Door1){
		UartTx(0x20,0x01, 0x11);
	}
	if (Door == &Door2){
		UartTx(0x20,0x02, 0x11);
	}
}
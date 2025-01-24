#ifndef __DOOR__
#define __DOOR__
#include "main.h"
#include "Motor.h"
#include "Ultrasonic.h"

typedef enum DoorPos_t{
	FRONT,
	REAR
}DoorPos;

typedef enum DoorStatus_t{
	OPEN,
	RUNNING,
	END
}DoorStatus;


typedef struct DoorInfo_t{
	MotorInfo *ServoMotor;
	DoorPos DoorPos;
	GPIO_TypeDef *UltraPort;
	uint16_t UltraPin;
	uint8_t LockFlag;
	uint8_t UnlockFlag;
	uint8_t OpenFlag;
	uint8_t CloseFlag;
	uint8_t IsObstacle;
	DoorStatus DoorStatus;
}DoorInfo;

extern DoorInfo Door1;
extern DoorInfo Door2;

void DoorInit(DoorInfo *Door, MotorInfo *ServoMotor);
void DoorActuate(DoorInfo *Door);
void DoorOpen(DoorInfo *Door);
void DoorClose(DoorInfo *Door);

void DoorLockCommand(void);

void DoorUnlockCommand(void);

void DoorCloseCommand(DoorInfo *Door);

void DoorOpenCommand(DoorInfo *Door);
#endif
#include "stm32l0xx_hal.h"
#include "Door.h"

/*
Operation : Check the current status when event(interrupt) occurs that controls the door
Input : door_t*, door_operation_t
Output : -1 ~ 1 integer value according to operation result
	-1 : Door is busy
	0 : OK
	1 : Unable to execute given operation (already opened/closed, etc.)
*/
int8_t CheckDoor(door_t* door, door_operation_t operation)
{
	// Target door is busy
	if(door->status != Idle)
	{
		return -1;	// Busy condition return
	}
	
	// Check current status & operation
	int8_t result = 1;
	switch(operation)
	{
		case DoorOpen:
			result = (door->door == Open);		// Door is already opened?
			break;
		case DoorClose:
			result = (door->door == Close);		// Door is already closed?
			break;
		case DoorLock:
			result = (door->door == Open || door->lock == Lock);		// Door is opened or already locked?
			break;
		case DoorUnlock:
			result = (door->lock == Unlock);	// Door is already unlocked?
			break;
		default:
			result = 1;												// Unknown operation
	}
	
	return result;
}

/*
Operation : Control automatic door
Input : door_seat_t(location), operation_t(Open/Close/Lock/Unlock)
*/
door_t* DoorControl(door_t* door, door_operation_t operation)
{
	// Change door condition to running
	door->status = Running;
	
	switch(operation)
	{
		case DoorOpen:
			if(door->lock == Lock)
				ControlLockUnlock(door, Unlock);			// State (0,0) -> (1,0)
			//HAL_Delay(15);												// Wait while unlocked
			ControlOpen(door, 0);										// State (0,1) -> (1,1)
			break;
		case DoorClose:
			if(ControlClose(door) == 0)							// State (1,1) -> (1,0)
			{
				//HAL_Delay(15);											// Wait while locked
				ControlLockUnlock(door, Lock);				// State (1,0) -> (0,0)
			}
			break;
		case DoorLock:
			ControlLockUnlock(door, Lock);					// State (1,0) -> (0,0)
			break;
		case DoorUnlock:
			ControlLockUnlock(door, Unlock);				// State (0,0) -> (1,0)
			break;
	}
	
	// Change door condition to CoolDown
	door->status = CoolDown;
	
	//HAL_TIM_Base_Start_IT(&htim6);						// 3 sec required
	return door;
}

// Lock/Unlock operation
void ControlLockUnlock(door_t* door, door_lock_t state)
{
	uint16_t Lock_Pin = 0;
	
	door->lock = state;								// Lock state change
	
	// Change LED
	Lock_Pin = (door->seat == Front ? Lock_Front_Pin : Lock_Back_Pin);
	LockToLED(Lock_Pin, state);
}

// DoorOpen operation
void ControlOpen(door_t* door, int stuck)
{
	volatile double distance = 0;
	uint16_t Trig_Pin = (door->seat == Front ? Trig_Front_Pin : Trig_Back_Pin);
	
	distance = Ultrasonic(Trig_Pin);
	
	/* yeonwoo's code */
}

// DoorClose operation
uint8_t ControlClose(door_t* door)
{
	int dist = 0, threshold = 0, tucked = 0;
	while(dist < threshold)
	{
		/*
		tucked = Motor(Close);
		
		if(tucked)									// Door tucked
		{
			// reset all motor timer
			OperationOpen(seat, 1)		// open -f (state (x,x) -> (1,1))
			break;
		}
		*/
	}
	
	door->lock = Lock;
	return 0;
}
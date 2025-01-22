#ifndef __DOOR__H
#define __DOOR__H

#include "Ultrasonic.h"
#include "Motor.h"
#include "DoorIo.h"

/* ---------- Typedef ---------- */
// Front/Back
typedef enum DoorSeat
{
	Front,
	Back
}door_seat_t;

// Closed/Opened
typedef enum DoorState
{
	Close,
	Open
}door_state_t;

// Idle/Running/Cooldown status
typedef enum DoorStatus
{
	Idle,
	Running,
	CoolDown,
}door_status_t;

// Automatic door struct
typedef struct AutomaticDoor
{
	door_seat_t seat;
	door_state_t door;
	door_lock_t lock;
	door_status_t status;
}door_t;

// Door operations
typedef enum DoorOperation
{
	DoorLock,
	DoorUnlock,
	DoorOpen,
	DoorClose
}door_operation_t;
/* ---------- Typedef ---------- */

/* ---------- Function Declaration ---------- */
int8_t CheckDoor(door_t* door, door_operation_t operation);
door_t* DoorControl(door_t* door, door_operation_t operation);

void ControlOpen(door_t* door, int stuck);
uint8_t ControlClose(door_t* door);
void ControlLockUnlock(door_t* door, door_lock_t state);
/* ---------- Function Declaration ---------- */
#endif
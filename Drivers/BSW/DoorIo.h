#ifndef __DOORIO__H
#define __DOORIO__H

#include <stdint.h>				// uint(x)_t style

/* ---------- Define ---------- */
// EXTI Door switch (No manual function / button push = falling edge)
#define Switch_GPIO_Port	GPIOB
#define Switch_Open_Pin		GPIO_PIN_12						// CN10 right 8
#define Switch_Close_Pin	GPIO_PIN_13						// CN10 right 15
#define Switch_Lock_Pin		GPIO_PIN_14						// CN10 right 14
#define Switch_Unlock_Pin	GPIO_PIN_15						// CN10 right 13

// LED Door lock (Lock = LED on / Unlock = LED off)
#define Lock_GPIO_Port 		GPIOB
#define Lock_Front_Pin		GPIO_PIN_3						// (CN9) D3
#define Lock_Back_Pin 		GPIO_PIN_10						// (CN9) D6
/* ---------- Define ---------- */

/* ---------- Typedef ---------- */
// Door Locked/Unlocked
typedef enum LockState
{
	Lock,
	Unlock
}door_lock_t;
/* ---------- Typedef ---------- */

/* ---------- Function Declaration ---------- */
// Change LED depending on lock state
void LockToLED(uint16_t Lock_Pin, door_lock_t state);
/* ---------- Function Declaration ---------- */
#endif
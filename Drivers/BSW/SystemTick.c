#include "main.h"
#include "SystemTick.h"


stTimeCnt TimeCnt;
stSchedulinginfo SchedulingInfo;
volatile uint8_t UltrasonicFlag_100ms;

void HAL_SYSTICK_Callback(void){
	SchedulingInfo.u8nuScheduling1msFlag = 1u;
	TimeCnt.u32nuCnt1ms++;
	if (TimeCnt.u32nuCnt1ms % 10 == 0){
		SchedulingInfo.u8nuScheduling10msFlag = 1u;
		TimeCnt.u32nuCnt10ms++;
	}
	if (TimeCnt.u32nuCnt1ms % 100 == 0){
		SchedulingInfo.u8nuScheduling100msFlag = 1u;
		//UltrasonicFlag_100ms = 1u;
		TimeCnt.u32nuCnt100ms++;
	}
	if (TimeCnt.u32nuCnt1ms % 1000 == 0){
		SchedulingInfo.u8nuScheduling1000msFlag = 1u;
		TimeCnt.u32nuCnt1000ms++;	
	}
}





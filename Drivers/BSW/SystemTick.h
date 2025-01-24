#ifndef __SYSTEMTICK__
#define __SYSTEMTICK__

#include "main.h"

typedef struct
{
    uint32_t u32nuCnt1ms;
    uint32_t u32nuCnt10ms;
    uint32_t u32nuCnt100ms;
    uint32_t u32nuCnt1000ms;
}stTimeCnt;

typedef struct stSchedulinginfo_t{
	uint8_t u8nuScheduling1msFlag;
	uint8_t u8nuScheduling10msFlag;
	uint8_t u8nuScheduling100msFlag;
	uint8_t u8nuScheduling1000msFlag;
}stSchedulinginfo;

void HAL_SYSTICK_Callback(void);

extern stTimeCnt TimeCnt;
extern stSchedulinginfo SchedulingInfo;

#endif
#ifndef __UART__
#define __UART__
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t uart_rcvbuf;
extern uint8_t uart1_tx_data[4];

void UARTInit(void);
void UART_TxData(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void UartTxBufferClear(void);
void UartTx(uint8_t Buffer0,uint8_t Buffer1,uint8_t Buffer2);
#endif
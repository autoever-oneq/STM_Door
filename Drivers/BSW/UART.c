#include "main.h"
#include "UART.h"
#include "Door.h"
#include "Motor.h"
#include <stdlib.h>


uint8_t uart1_rx_data[2];
uint8_t uart1_tx_data[4];
uint8_t uart2_rcvbuf = 0;


void UARTInit(void){
	HAL_UART_Receive_IT(&huart1,uart1_rx_data, 2);
	HAL_UART_Receive_IT(&huart2,&uart2_rcvbuf, 1);
}

void UART_TxData(void){
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	
	if(UartHandle == &huart1) {
		
		uint8_t Buffer1 = uart1_rx_data[0];
		uint8_t Buffer2 = uart1_rx_data[1];
		
		if ((Buffer1 == 0x10)&&(Buffer2 == 0x03)){
			DoorLockCommand();
		}
		if ((Buffer1 == 0x11)&&(Buffer2 == 0x03)){
			DoorUnlockCommand();
		}
		if ((Buffer1 == 0x12)&&(Buffer2 ==0x01)){
			DoorOpenCommand(&Door1);
		}
		if ((Buffer1 == 0x12)&&(Buffer2 ==0x02)){
			DoorOpenCommand(&Door2);
		}
		if ((Buffer1 == 0x13)&&(Buffer2 ==0x01)){
			DoorCloseCommand(&Door1);
		}
		if ((Buffer1 == 0x13)&&(Buffer2 ==0x02)){
			DoorCloseCommand(&Door2);
		}	
	}
	
	if(UartHandle == &huart2) {
		HAL_UART_Transmit(&huart2, &uart2_rcvbuf, 1, 300);
		if (uart2_rcvbuf == '1'){
			DoorUnlockCommand();
		}
		if (uart2_rcvbuf == '2'){
			DoorLockCommand();
		}		
		if (uart2_rcvbuf == '3'){
			DoorOpenCommand(&Door1);	
		}		
		if (uart2_rcvbuf == '4'){
			DoorCloseCommand(&Door1);	
		}
					
		if (uart2_rcvbuf == 'e'){
			DoorOpenCommand(&Door2);			
		}		
		if (uart2_rcvbuf == 'r'){
			DoorCloseCommand(&Door2);		
		}	
		uart2_rcvbuf = 0;
	}
	UARTInit();
}
void UartTx(uint8_t Buffer0, uint8_t Buffer1, uint8_t Buffer2){
	uart1_tx_data[0] = Buffer0;
	uart1_tx_data[1] = Buffer1;
	uart1_tx_data[2] = Buffer2;
	uart1_tx_data[3] = 0xFF;
	HAL_UART_Transmit(&huart1,uart1_tx_data,4,300);
}

void UartTxBufferClear(void){
	uart1_tx_data[0] = 0;
	uart1_tx_data[1] = 0;
	uart1_tx_data[2] = 0;
}
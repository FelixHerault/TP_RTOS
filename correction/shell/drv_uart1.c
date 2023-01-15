/*
 * drv_uart1.c
 *
 *  Created on: 7 nov. 2022
 *      Author: laurentf
 */

#include "drv_uart1.h"
#include "cmsis_os.h"

#include "usart.h"
#include "gpio.h"
#include <stdio.h>


extern SemaphoreHandle_t sem_uart;

uint8_t drv_uart1_receive(char * pData, uint16_t size)
{
	//printf("je suis dans huart receive\r\n");
	HAL_UART_Receive_IT(&huart1, (uint8_t*)(pData), size);
	//printf("je suis dans huart receive\r\n");
	xSemaphoreTake(sem_uart,portMAX_DELAY);

	return 0;	// Life's too short for error management
}

uint8_t drv_uart1_transmit(const char * pData, uint16_t size)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)pData, size, HAL_MAX_DELAY);

	return 0;	// Srsly, don't do that kids
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


  if(huart==&huart1){
	 // printf("je suis dans Callback\r\n");
	  BaseType_t xHigherPriorityTaskWoken;
	  xHigherPriorityTaskWoken = pdFALSE;
	 // printf("je vais donner le semaphore\r\n");
	 xSemaphoreGiveFromISR(sem_uart, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	//  printf("j'ai donné le semaphore\r\n");
  }
}

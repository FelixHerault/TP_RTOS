/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "shell.h"
#include "drv_uart1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_shell_t h_shell;
TaskHandle_t hLed;
TaskHandle_t hShell;
TaskHandle_t hb1;
TaskHandle_t hb2;
TaskHandle_t hb3;
TaskHandle_t hb4;
//TaskHandle_t hSpam;
SemaphoreHandle_t sem_led;
//SemaphoreHandle_t sem_spam;
SemaphoreHandle_t sem_uart;
char* msg_spam[1];
int nb_spam;

int delay = 100;
int led_period;

#define SHELL_STACK_SIZE 512
#define SHELL_PRIORITY 3
#define LED_STACK_SIZE 512
#define LED_PRIORITY 2
#define LED_STACK_SIZE1 512 // pour la creation de tache bidon
//#define SPAM_STACK_SIZE 1024
//#define SPAM_PRIORITY 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

int fonction(h_shell_t * h_shell, int argc, char ** argv)
{
	int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "Je suis une fonction bidon\r\n");
	h_shell->drv.transmit(h_shell->print_buffer, size);

	return 0;
}

void task_led(void * unused){ // tache qui permet de faire clignoter la led
	if(xSemaphoreTake(sem_led,portMAX_DELAY)== pdTRUE){
		for(;;){
			if(led_period <= 0){
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
				vTaskDelay(500); // il s'agit d'une valeur arbitraire qui permet de ne pas appeler la tache toute les ms
			}
			else{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
				vTaskDelay(led_period);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
				vTaskDelay(led_period);
			}
		}
	}
}

// pour la creation de tache bidon
void task_b1(void * unused){ // tache qui permet de faire clignoter la led

}
void task_b2(void * unused){ // tache qui permet de faire clignoter la led

}
void task_b3(void * unused){ // tache qui permet de faire clignoter la led

}
void task_b4(void * unused){ // tache qui permet de faire clignoter la led

}



int led(h_shell_t * h_shell, int argc, char ** argv) // fonction qui appelle la tache qui fait clignoter la led, et renvoie l'état sur le shell
{
	if (argc != 2){
		int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "ERROR y a pas le bon nb\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);

		return -1;
	}

	else {
		led_period = atoi(argv[1]);
		xSemaphoreGive(sem_led);
		if(led_period > 0){
			int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "led clignote a une periode de %d ms\r\n", led_period);
			h_shell->drv.transmit(h_shell->print_buffer, size);
		}
		else {
			int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "la led est eteinte\r\n");
			h_shell->drv.transmit(h_shell->print_buffer, size);
		}
	}
	return 0;
}



//void task_spam( void * pvParameters){
//	h_shell_t * hshell = &h_shell;
//	if(xSemaphoreTake(sem_spam,portMAX_DELAY)== pdTRUE){
//		for(int i = 0; i < nb_spam; i++){
//			int size = snprintf (hshell->print_buffer, BUFFER_SIZE, "%s\r\n", msg_spam[0]);
//			hshell->drv.transmit(hshell->print_buffer, size);
//		}
//	}
//}
//
//int spam(h_shell_t * h_shell, int argc, char ** argv) {
//	if (argc != 3){
//		int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "ERROR y a pas le bon nb\r\n");
//		h_shell->drv.transmit(h_shell->print_buffer, size);
//
//		return -1;
//	}
//	else {
//		msg_spam[0] = argv[1];
//		nb_spam = atoi(argv[2]);
//		xSemaphoreGive(sem_spam);
//	}
//	return 0;
//}

void task_shell(void * unused){ // tache qui gère le shell
	shell_init(&h_shell);
	shell_add(&h_shell, 'f', fonction, "Une fonction inutile");
	shell_add(&h_shell, 'l', led,"clignotement de la led");
	//shell_add(&h_shell, 's', spam,"spam");
	shell_run(&h_shell);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	h_shell.drv.receive = drv_uart1_receive;
	h_shell.drv.transmit = drv_uart1_transmit;
	sem_led = xSemaphoreCreateBinary();
	sem_uart = xSemaphoreCreateBinary();
	//sem_spam = xSemaphoreCreateBinary();
	BaseType_t ret;

	ret=xTaskCreate(task_shell, "SHELL", SHELL_STACK_SIZE, NULL, SHELL_PRIORITY, &hShell);
	configASSERT(ret == pdPASS);

	ret=xTaskCreate(task_led, "LED", LED_STACK_SIZE, NULL, LED_PRIORITY, &hLed);
	configASSERT(ret == pdPASS);

//	ret=xTaskCreate(task_spam, "SPAM", SPAM_STACK_SIZE, (void * ) 1, SPAM_PRIORITY, &hSpam);
//	configASSERT(ret == pdPASS);


	// creation de taches bidons
	ret=xTaskCreate(task_b1, "B1", LED_STACK_SIZE1, NULL, LED_PRIORITY, &hb1);
	configASSERT(ret == pdPASS);
	ret=xTaskCreate(task_b2, "B2", LED_STACK_SIZE1, NULL, LED_PRIORITY, &hb2);
	configASSERT(ret == pdPASS);
	ret=xTaskCreate(task_b3, "B3", LED_STACK_SIZE1, NULL, LED_PRIORITY, &hb3);
	configASSERT(ret == pdPASS);
	ret=xTaskCreate(task_b4, "B4", LED_STACK_SIZE1, NULL, LED_PRIORITY, &hb4);
	configASSERT(ret == pdPASS);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

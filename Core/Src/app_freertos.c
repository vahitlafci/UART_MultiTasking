/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stringOp.h"
#include "std_types.h"
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
/* USER CODE BEGIN Variables */
#define BUFFER_LENGTH 50
/*extern variables*/
extern uint8_t uartItFlag;
extern uint8_t buffer[BUFFER_LENGTH];
extern uint8_t rcvd_data;
extern uint8_t cntr;
extern uint8_t rcvd_complete;
extern uint8_t start;
extern uint8_t end;
extern UART_HandleTypeDef hlpuart1;
/*shared variables*/
operationType current_operation = OP_INVALID;
uint16_t ledOnTime = 500;
uint16_t ledOffTime = 500;
uint32_t baud = 115200;
uint8_t ledEvent = 0;
uint8_t uartEvent = 0;
/*static variables*/
static ledOperationsStateType led_state = STATE_LED_ON_INITIAL;
static uint32_t ledCounter;
static uint8_t taskStopped = 1;
static uint8_t echoData[100];
static uint8_t echoLength = 0;
static uint8_t echoFlag = 0;
static uint8_t echoData[100];
/* USER CODE END Variables */
osThreadId LedTaskHandle;
osThreadId UartTaskHandle;
osThreadId echoTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedTask_func(void const * argument);
void UartTask_Func(void const * argument);
void echoTask_Func(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LedTask */
  osThreadDef(LedTask, LedTask_func, osPriorityNormal, 0, 256);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, UartTask_Func, osPriorityIdle, 0, 1500);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  /* definition and creation of echoTask */
  osThreadDef(echoTask, echoTask_Func, osPriorityIdle, 0, 256);
  echoTaskHandle = osThreadCreate(osThread(echoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LedTask_func */
/**
 * @brief  Function implementing the LedTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedTask_func */
void LedTask_func(void const * argument)
{
  /* USER CODE BEGIN LedTask_func */
	/* Infinite loop */
	for (;;)
	{
		if (ledEvent == 1)
		{
			ledEvent = 0;
			led_state = STATE_LED_ON_INITIAL;
		}
		switch (led_state)
		{
		case STATE_LED_ON_INITIAL:
			ledCounter = ledOnTime;
			led_state = STATE_LED_ON_ONGOING;
			break;
		case STATE_LED_OFF_INITIAL:
			ledCounter = ledOffTime;
			led_state = STATE_LED_OFF_ONGOING;
			break;
		case STATE_LED_OFF_ONGOING:
			ledCounter--;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			if (ledCounter == 0)
			{
				led_state = STATE_LED_ON_INITIAL;
			}
			break;
		case STATE_LED_ON_ONGOING:
			ledCounter--;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			if (ledCounter == 0)
			{
				led_state = STATE_LED_OFF_INITIAL;
			}
			break;
		}
		osDelay(1);
	}
  /* USER CODE END LedTask_func */
}

/* USER CODE BEGIN Header_UartTask_Func */
/**
 * @brief Function implementing the UartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartTask_Func */
void UartTask_Func(void const * argument)
{
  /* USER CODE BEGIN UartTask_Func */
	/* Infinite loop */
	for (;;)
	{
		if (uartItFlag)
		{
			uartItFlag = 0;
			start = cntr;
			for (int i = start; i < start + BUFFER_LENGTH; i++)
			{
				rcvd_data = buffer[i];
				if (rcvd_data == 10)
				{
					rcvd_complete = 1;
					end = cntr;
					cntr++;
					break;
				}
				cntr++;
			}
		}

		if (rcvd_complete)
		{
			uint8_t length = end - start;
			uint8_t tmpData[length];
			for (int i = 0; i < length; i++)
			{
				tmpData[i] = buffer[start + i];
				echoData[i] = buffer[start + i];
			}
			echoLength = length;
			rcvd_complete = 0;
			strOp(&tmpData);
			echoFlag = 1;

			switch (current_operation)
			{
			case OP_STOP:
				if (!taskStopped)
				{
					vTaskSuspend(echoTaskHandle);
					taskStopped = 1;
				}
				break;
			case OP_INVALID:
				printf("E_N_OK\n");
				break;
			case OP_START:
				if (taskStopped)
				{
					vTaskResume(echoTaskHandle);
					taskStopped = 0;
				}
				echoFlag = 1;
			case OP_BAUD:
				echoFlag = 1;
				HAL_UART_Abort_IT(&hlpuart1);
				HAL_UART_DeInit(&hlpuart1);
				MX_LPUART1_UART_Init();
				HAL_UART_Receive_IT(&hlpuart1, buffer, BUFFER_LENGTH);
			default:
				echoFlag = 1;
				break;
			}
		}
		osDelay(1);
	}
  /* USER CODE END UartTask_Func */
}

/* USER CODE BEGIN Header_echoTask_Func */
/**
* @brief Function implementing the echoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_echoTask_Func */
void echoTask_Func(void const * argument)
{
  /* USER CODE BEGIN echoTask_Func */
  /* Infinite loop */
  for(;;)
  {
	if(echoFlag) {
		echoFlag = 0;
		HAL_UART_Transmit(&hlpuart1,echoData, echoLength, 0xFFFF);
		printf("\n");
	}
    osDelay(20);
  }
  /* USER CODE END echoTask_Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


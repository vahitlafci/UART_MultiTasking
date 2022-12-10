/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
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
extern UART_HandleTypeDef huart1;
/*shared variables*/
operationType current_operation = OP_INVALID;
uint16_t ledOnTime = 500;
uint16_t ledOffTime = 500;
uint32_t baud = 115200;
uint8_t ledEvent = 0;
uint8_t uartEvent = 0;
/*static variables*/
static ledOperationsStateType led_state = STATE_LED_ON_INITIAL;
static ledCounter;
/* USER CODE END Variables */
osThreadId LedTaskHandle;
osThreadId UartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedTask_func(void const *argument);
void UartTask_func(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  osThreadDef(UartTask, UartTask_func, osPriorityLow, 0, 1024);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

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
void LedTask_func(void const *argument)
{
  /* USER CODE BEGIN LedTask_func */
  /* Infinite loop */
  for (;;)
  {
    if(ledEvent==1) {
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
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
      if(ledCounter == 0) {
        led_state = STATE_LED_ON_INITIAL;
      }
      break;
    case STATE_LED_ON_ONGOING:
      ledCounter--;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
      if(ledCounter == 0) {
        led_state = STATE_LED_OFF_INITIAL;
      }
      break;
    }
    osDelay(1);
  }
  /* USER CODE END LedTask_func */
}

/* USER CODE BEGIN Header_UartTask_func */
/**
 * @brief Function implementing the UartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartTask_func */
void UartTask_func(void const *argument)
{
  /* USER CODE BEGIN UartTask_func */
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
      uint8_t echoData[length];
      for (int i = 0; i < length; i++)
      {
        tmpData[i] = buffer[start + i];
        echoData[length] = buffer[start + i];
      }
      strOp(&tmpData);
      rcvd_complete = 0;
      if (current_operation != OP_STOP) // TODO(VahitL) : This means, echo will proceed if the last received command was not stop. This will block echo function. A new task need to be added in the future for this operation.
        echoFunc(&echoData);
      if(current_operation == OP_BAUD){
    	 HAL_UART_Abort_IT(&huart1);
    	 HAL_UART_DeInit(&huart1);
    	 MX_USART1_UART_Init();
      }
    }
    printf("test\n");
    osDelay(1000);
  }
  /* USER CODE END UartTask_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

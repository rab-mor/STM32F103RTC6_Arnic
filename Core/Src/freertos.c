/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos on the F103 on the main ARNIC Board
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_shared.h"
#include "F10Slave.h"
#include "spi.h"
#include "f103_tasks.h"
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

osThreadId_t RelayTaskHandle;
static const osThreadAttr_t RelayTask_attributes = {
	.name = "RelayTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityAboveNormal
};


osThreadId_t MeasureTaskHandle;
static const osThreadAttr_t MeasureTask_attributes = {
	.name = "MeasureTask",
	.stack_size = 384 * 4,
	.priority = (osPriority_t) osPriorityNormal
};


osThreadId_t SupervisorTaskHandle;
static const osThreadAttr_t SupervisorTask_attributes = {
	.name = "SupervisorTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityHigh,
};

/* USER CODE END Variables */
/* Definitions for LinkTask */
osThreadId_t LinkTaskHandle;
const osThreadAttr_t LinkTask_attributes = {
  .name = "LinkTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for relay_cmd_q */
osMessageQueueId_t relay_cmd_qHandle;
const osMessageQueueAttr_t relay_cmd_q_attributes = {
  .name = "relay_cmd_q"
};
/* Definitions for event_q */
osMessageQueueId_t event_qHandle;
const osMessageQueueAttr_t event_q_attributes = {
  .name = "event_q"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLinkTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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

  /* Create the queue(s) */
  /* creation of relay_cmd_q */
  relay_cmd_qHandle = osMessageQueueNew (8, sizeof(relay_cmd_t), &relay_cmd_q_attributes);

  /* creation of event_q */
  event_qHandle = osMessageQueueNew (8, sizeof(link_event_t), &event_q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LinkTask */
  LinkTaskHandle = osThreadNew(StartLinkTask, NULL, &LinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  RelayTaskHandle   = osThreadNew(RelayTask_Run,   NULL, &RelayTask_attributes);
  MeasureTaskHandle = osThreadNew(MeasureTask_Run, NULL, &MeasureTask_attributes);
  SupervisorTaskHandle = osThreadNew(SupervisorTask_Run, NULL, &SupervisorTask_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLinkTask */
/**
  * @brief  Function implementing the LinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLinkTask */
void StartLinkTask(void *argument)
{
  /* USER CODE BEGIN StartLinkTask */
	F10Slave_CsExtiInit();
	F10Slave_Init(&hspi2);
  /* Infinite loop */
  for(;;)
  {
	F10Slave_Service();
  }
  /* USER CODE END StartLinkTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


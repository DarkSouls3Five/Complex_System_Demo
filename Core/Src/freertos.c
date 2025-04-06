/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
osThreadId LED_GreenHandle;
osThreadId MotorTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ServoTaskHandle;
osThreadId Infrared_DetectHandle;
osThreadId Ultrasonic_DeteHandle;
osThreadId TranslateTaskHandle;
osThreadId ModeSetTaskHandle;
osThreadId buzzer_taskHandle;
osThreadId LedFlowTaskHandle;
osThreadId Yaw_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Servo_Task(void const * argument);
void infrared_task(void const * argument);
void ultrasonic_task(void const * argument);
void translate_task(void const * argument);
void mode_set_task(void const * argument);
void Buzzer_Task(void const * argument);
void led_RGB_flow_task(void const * argument);
void yaw_control_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, Servo_Task, osPriorityHigh, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of Infrared_Detect */
  osThreadDef(Infrared_Detect, infrared_task, osPriorityIdle, 0, 128);
  Infrared_DetectHandle = osThreadCreate(osThread(Infrared_Detect), NULL);

  /* definition and creation of Ultrasonic_Dete */
  osThreadDef(Ultrasonic_Dete, ultrasonic_task, osPriorityIdle, 0, 128);
  Ultrasonic_DeteHandle = osThreadCreate(osThread(Ultrasonic_Dete), NULL);

  /* definition and creation of TranslateTask */
  osThreadDef(TranslateTask, translate_task, osPriorityIdle, 0, 128);
  TranslateTaskHandle = osThreadCreate(osThread(TranslateTask), NULL);

  /* definition and creation of ModeSetTask */
  osThreadDef(ModeSetTask, mode_set_task, osPriorityIdle, 0, 128);
  ModeSetTaskHandle = osThreadCreate(osThread(ModeSetTask), NULL);

  /* definition and creation of buzzer_task */
  osThreadDef(buzzer_task, Buzzer_Task, osPriorityBelowNormal, 0, 128);
  buzzer_taskHandle = osThreadCreate(osThread(buzzer_task), NULL);

  /* definition and creation of LedFlowTask */
  osThreadDef(LedFlowTask, led_RGB_flow_task, osPriorityNormal, 0, 128);
  LedFlowTaskHandle = osThreadCreate(osThread(LedFlowTask), NULL);

  /* definition and creation of Yaw_task */
  osThreadDef(Yaw_task, yaw_control_task, osPriorityIdle, 0, 128);
  Yaw_taskHandle = osThreadCreate(osThread(Yaw_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Servo_Task */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Task */
__weak void Servo_Task(void const * argument)
{
  /* USER CODE BEGIN Servo_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Servo_Task */
}

/* USER CODE BEGIN Header_infrared_task */
/**
* @brief Function implementing the Infrared_Detect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_infrared_task */
__weak void infrared_task(void const * argument)
{
  /* USER CODE BEGIN infrared_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END infrared_task */
}

/* USER CODE BEGIN Header_ultrasonic_task */
/**
* @brief Function implementing the Ultrasonic_Dete thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultrasonic_task */
__weak void ultrasonic_task(void const * argument)
{
  /* USER CODE BEGIN ultrasonic_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ultrasonic_task */
}

/* USER CODE BEGIN Header_translate_task */
/**
* @brief Function implementing the TranslateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_translate_task */
__weak void translate_task(void const * argument)
{
  /* USER CODE BEGIN translate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END translate_task */
}

/* USER CODE BEGIN Header_mode_set_task */
/**
* @brief Function implementing the ModeSetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mode_set_task */
__weak void mode_set_task(void const * argument)
{
  /* USER CODE BEGIN mode_set_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mode_set_task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the buzzer_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void const * argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_led_RGB_flow_task */
/**
* @brief Function implementing the LedFlowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_RGB_flow_task */
__weak void led_RGB_flow_task(void const * argument)
{
  /* USER CODE BEGIN led_RGB_flow_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_RGB_flow_task */
}

/* USER CODE BEGIN Header_yaw_control_task */
/**
* @brief Function implementing the Yaw_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_yaw_control_task */
__weak void yaw_control_task(void const * argument)
{
  /* USER CODE BEGIN yaw_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END yaw_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void can_filter_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t	infrared_return;//表征红外传感器检测状态的变量

uint32_t time_ms = 0;
int time_second = 0;
uint8_t sbus_buf[18];

//超声传感器相关定义
Ultrasonic_Data_t Ultrasonic_Data[3]; // 3 个超声波传感器的数据

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
	infrared_return=0;//初始化红外传感器
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_CAN2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//定时器开始工作
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);		//开启蜂鸣器对应的定时器4通道3
	HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	//can过滤器
	can_filter_init();
	HAL_UART_Receive_DMA(&huart3,sbus_buf,18);
	
	HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_1);   //开启TIM8的通道1，并且开启捕获中断
  HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_2);   //开启TIM8的通道2，并且开启捕获中断	
  HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_3);   //开启TIM8的通道3，并且开启捕获中断
  __HAL_TIM_ENABLE_IT(&htim8,TIM_IT_UPDATE);   //使能更新中断
	

                        
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//can过滤器
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

//超声传感器输入捕获中断
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        uint8_t channel = 0;
				uint32_t Channel_x = TIM_CHANNEL_1;//后续需要的通道变量
        // 判断是哪个通道触发了捕获事件
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // CH1，超声1
				{
					Channel_x = TIM_CHANNEL_1;//根据channel值转换为相应的通道变量
					channel = 0;
				}
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 
				{	
					Channel_x = TIM_CHANNEL_2;
					channel = 1; // CH2，超声2
				}
				else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) 
				{	
					Channel_x = TIM_CHANNEL_3;
					channel = 2; // CH3，超声3
				}

        if (Ultrasonic_Data[channel].Is_First_Captured == 0) // 第一次捕获（上升沿）
        {
            Ultrasonic_Data[channel].IC_Val1 = HAL_TIM_ReadCapturedValue(htim, Channel_x); // 读取捕获值
            Ultrasonic_Data[channel].Is_First_Captured = 1; // 标记第一次捕获已完成
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, Channel_x, TIM_INPUTCHANNELPOLARITY_FALLING); // 设置捕获极性为下降沿
        }
        else // 第二次捕获（下降沿）
        {
            Ultrasonic_Data[channel].IC_Val2 = HAL_TIM_ReadCapturedValue(htim, Channel_x); // 读取捕获值
            __HAL_TIM_SET_COUNTER(htim, 0); // 重置计数器

            // 计算时间差
            uint32_t Difference = (Ultrasonic_Data[channel].IC_Val2 > Ultrasonic_Data[channel].IC_Val1) ?
                                  (Ultrasonic_Data[channel].IC_Val2 - Ultrasonic_Data[channel].IC_Val1) :
                                  ((0xFFFF - Ultrasonic_Data[channel].IC_Val1) + Ultrasonic_Data[channel].IC_Val2);

            // 计算距离（单位：cm）
            Ultrasonic_Data[channel].Distance = Difference * 0.034 / 2;

            // 重置捕获标志
            Ultrasonic_Data[channel].Is_First_Captured = 0;

            // 恢复捕获极性为上升沿
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, Channel_x, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

//超声传感器引脚输入电平
void Ultrasonic_Trigger(GPIO_TypeDef *Trig_GPIO_Port, uint16_t Trig_Pin)
{
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET); // 设置 Trig 引脚为高电平
    osDelay(10);// 延时约 10 μs
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET); // 设置 Trig 引脚为低电平
}
/* USER CODE END 4 */

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

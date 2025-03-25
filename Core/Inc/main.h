/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Trig2_Pin GPIO_PIN_0
#define Trig2_GPIO_Port GPIOF
#define Trig1_Pin GPIO_PIN_1
#define Trig1_GPIO_Port GPIOF
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define Infrared2_Pin GPIO_PIN_13
#define Infrared2_GPIO_Port GPIOE
#define Infrared1_Pin GPIO_PIN_11
#define Infrared1_GPIO_Port GPIOE
#define Infrared3_Pin GPIO_PIN_14
#define Infrared3_GPIO_Port GPIOE
#define Infrared4_Pin GPIO_PIN_12
#define Infrared4_GPIO_Port GPIOB
#define Trig3_Pin GPIO_PIN_14
#define Trig3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern uint32_t time_ms;
extern int time_second;

void Ultrasonic_Trigger(GPIO_TypeDef *Trig_GPIO_Port, uint16_t Trig_Pin);

//定义超声传感器数据存储变量
typedef struct
{
    uint32_t IC_Val1; // 第一次捕获值
    uint32_t IC_Val2; // 第二次捕获值
    uint8_t Is_First_Captured; // 是否是第一次捕获
    float Distance; // 计算的距离值
} Ultrasonic_Data_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

#include "LEDTask.h"
#include "main.h"
#include "cmsis_os.h"

void LED_Blue_task(void const * argument)
{
	while(1){
		
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
		vTaskDelay(500);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
		vTaskDelay(500);
		
  }
}

void LED_Green_task(void const * argument)
{
	while(1){
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
		vTaskDelay(750);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);
		vTaskDelay(750);
  }
}





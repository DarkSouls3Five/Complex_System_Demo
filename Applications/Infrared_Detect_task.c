#include "main.h"
#include "cmsis_os.h"
#include "Infrared_Detect_task.h"

//外部引用infrared_return变量，该变量的值只应在本任务中被改变
extern uint8_t infrared_return;



/**
  * @brief          红外检测任务，判断接收到障碍物的是哪个传感器，并将infrared_return改变为相应的值
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void infrared_task(void const * argument)
{
	while(1)
	{
		//判断优先级A→B→C→D，2-4-3-1
		vTaskDelay(50);
		if(HAL_GPIO_ReadPin(Infrared2_GPIO_Port, Infrared2_Pin) == GPIO_PIN_RESET)        //当检测到红外避障传感器1(PE11)为低电平持续1s，即持续检测到物体1s
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared2_GPIO_Port, Infrared2_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=2;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }
		else if(HAL_GPIO_ReadPin(Infrared4_GPIO_Port, Infrared4_Pin) == GPIO_PIN_RESET)        //红外避障传感器2(PE13)
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared4_GPIO_Port, Infrared4_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=4;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }	
		else if(HAL_GPIO_ReadPin(Infrared3_GPIO_Port, Infrared3_Pin) == GPIO_PIN_RESET)        //红外避障传感器3(PE14)
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared3_GPIO_Port, Infrared3_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=3;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }
		else if(HAL_GPIO_ReadPin(Infrared1_GPIO_Port, Infrared1_Pin) == GPIO_PIN_RESET)        //红外避障传感器4(PB12)为低电平
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared1_GPIO_Port, Infrared1_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=1;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }		
	}
}



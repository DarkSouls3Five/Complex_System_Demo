#include "main.h"
#include "cmsis_os.h"
#include "Infrared_Detect_task.h"

//�ⲿ����infrared_return�������ñ�����ֵֻӦ�ڱ������б��ı�
extern uint8_t infrared_return;



/**
  * @brief          �����������жϽ��յ��ϰ�������ĸ�������������infrared_return�ı�Ϊ��Ӧ��ֵ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void infrared_task(void const * argument)
{
	while(1)
	{
		//�ж����ȼ�A��B��C��D��2-4-3-1
		vTaskDelay(50);
		if(HAL_GPIO_ReadPin(Infrared2_GPIO_Port, Infrared2_Pin) == GPIO_PIN_RESET)        //����⵽������ϴ�����1(PE11)Ϊ�͵�ƽ����1s����������⵽����1s
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared2_GPIO_Port, Infrared2_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=2;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }
		else if(HAL_GPIO_ReadPin(Infrared4_GPIO_Port, Infrared4_Pin) == GPIO_PIN_RESET)        //������ϴ�����2(PE13)
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared4_GPIO_Port, Infrared4_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=4;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }	
		else if(HAL_GPIO_ReadPin(Infrared3_GPIO_Port, Infrared3_Pin) == GPIO_PIN_RESET)        //������ϴ�����3(PE14)
	  {
		  
			vTaskDelay(1000);              
			if(HAL_GPIO_ReadPin(Infrared3_GPIO_Port, Infrared3_Pin) == GPIO_PIN_RESET)
			{
				infrared_return=3;		
				vTaskDelay(5000);
				infrared_return=0;					
			}
	  }
		else if(HAL_GPIO_ReadPin(Infrared1_GPIO_Port, Infrared1_Pin) == GPIO_PIN_RESET)        //������ϴ�����4(PB12)Ϊ�͵�ƽ
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



#include "main.h"
#include "cmsis_os.h"
#include "Usart1.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
//#include "MotorTask.h"

uint8_t num=0;

uint8_t buf[]="Helloword";
char Buf[20];

void Usart_Task()
{
	
	while(1)
	{	
//		if(motor_flag)
//		{
//			sprintf(Buf, "%d", num);
//			HAL_UART_Transmit(&huart1,(uint8_t *)Buf,strlen(Buf),1000);
//			vTaskDelay(500);
//			num++;
//		}
	}
}


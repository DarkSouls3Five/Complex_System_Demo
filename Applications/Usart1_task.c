#include "Usart1_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "MotorTask.h"

//uint8_t buf[]="hello_world\r\n";

char Buf[10];
int num=1;

void USART_Task()
{
	while(1)
	{
		if(motor_flag)
		{
			sprintf(Buf, "%d\n", num);
			HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
			vTaskDelay(500);
			num++;
		}
	}	
	
}


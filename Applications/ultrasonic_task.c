#include "main.h"
#include "cmsis_os.h"
#include "ultrasonic_task.h"

extern Ultrasonic_Data_t Ultrasonic_Data[3];
float distance1,distance2,distance3;
uint8_t mode_set_flag;

void ultrasonic_task(void const * argument)
{
	osDelay(1000);
	mode_set_flag = 0;
	while(1)
	{
			Ultrasonic_Trigger(Trig1_GPIO_Port, Trig1_Pin); // ���������� 1
      osDelay(50); // ��ʱ 50ms
      Ultrasonic_Trigger(Trig2_GPIO_Port, Trig2_Pin); // ���������� 2
      osDelay(50); // ��ʱ 50ms
			Ultrasonic_Trigger(Trig3_GPIO_Port, Trig3_Pin); // ���������� 3
      osDelay(50); // ��ʱ 50ms
			distance1 = Ultrasonic_Data[0].Distance;
			distance2 = Ultrasonic_Data[1].Distance;
			distance3 = Ultrasonic_Data[2].Distance;
		
			//ģʽ�ı��ʶ
			if( distance1 < 8.00f && distance2 < 8.00f )
				mode_set_flag = 1;
			else
				mode_set_flag = 0;				
		
	}
}

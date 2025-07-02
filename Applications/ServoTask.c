#include "ServoTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "mode_set_task.h"
#include "tim.h"

int servo_flag = 0;
extern uint8_t infrared_return;//�ⲿ���ú���״̬
extern uint8_t turn_set_flag;//������ת�����ʶ
uint16_t DC_Servo;

void servo_init()
{
	DC_Servo = DC_CLOSE;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
	osDelay(1000);
}

void Servo_Task(void const * argument)
{
//	vTaskDelay(SERVO_TASK_INIT_TIME);
	servo_init();
	while(1)
	{	
		if(turn_set_flag!=0)
		{
			if(DC_Servo == DC_CLOSE)//��ռ�ձ�Ϊ�ر�ֵ��˵���ս���Ҫ����״̬���ȵȺ�1�뱣֤�������ת��
				osDelay(1000);
			while(DC_Servo > DC_OPEN)//�����ʱռ�ձȽ�������
			{
				DC_Servo--;
				osDelay(10);				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
			}
		}
		
		else 
		{
			while(DC_Servo < DC_CLOSE)//�ر����ʱռ�ձȽ����С
			{
				DC_Servo++;
				osDelay(10);				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
			}
			
			//����ʱ������Ϊ�ر�ֵ
			DC_Servo = DC_CLOSE;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
		}
		
		vTaskDelay(10);
	}
}






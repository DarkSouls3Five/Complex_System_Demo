#include "ServoTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "mode_set_task.h"
#include "tim.h"

int servo_flag = 0;
extern uint8_t infrared_return;//外部引用红外状态
extern uint8_t turn_set_flag;//引用旋转方向标识
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
			if(DC_Servo == DC_CLOSE)//若占空比为关闭值，说明刚进入要开盖状态，先等候1秒保证箱体完成转动
				osDelay(1000);
			while(DC_Servo > DC_OPEN)//打开箱盖时占空比渐变增加
			{
				DC_Servo--;
				osDelay(10);				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
			}
		}
		
		else 
		{
			while(DC_Servo < DC_CLOSE)//关闭箱盖时占空比渐变减小
			{
				DC_Servo++;
				osDelay(10);				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
			}
			
			//其他时候，设置为关闭值
			DC_Servo = DC_CLOSE;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,DC_Servo);
		}
		
		vTaskDelay(10);
	}
}






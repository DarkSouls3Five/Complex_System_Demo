#include "ServoTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

int servo_flag = 0;
extern uint8_t infrared_return;//外部引用红外状态
void servo_init()
{

	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,100);
	osDelay(1000);
}

void Servo_Task(void const * argument)
{
//	vTaskDelay(SERVO_TASK_INIT_TIME);
	servo_init();
	while(1)
	{	
		if(infrared_return==0)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,75);
			osDelay(1000);
		}
		else if(infrared_return==1)//测试，当1号红外检测到障碍物时舵机旋转一定角度
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,125);
			osDelay(1000);
		}
		else
			osDelay(1000);//防止任务堵塞
	}
}






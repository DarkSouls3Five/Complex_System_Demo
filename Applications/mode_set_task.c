/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       mode_set_task.c/h
  * @brief      自由模式/工作模式切换任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     March-25-2024   Ignis             1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "user_lib.h"
#include "mode_set_task.h"


static void mode_init(garbage_mode_t *garbage_mode_init);
extern uint8_t mode_set_flag;
extern uint8_t turn_set_flag;
extern float distance1,distance2,distance3;

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void mode_set(garbage_mode_t *garbage_mode_set);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */


garbage_mode_t garbage_mode;
/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          横移机构任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void mode_set_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(MODE_SET_TASK_INIT_TIME);
    //chassis init
    //?????'??
    mode_init(&garbage_mode);
    while(1)
    {
			mode_set(&garbage_mode);                    //???????????g?
			vTaskDelay(MODE_SET_TIME_MS);
			garbage_mode.last_garbage_mode = garbage_mode.garbage_mode;
			

    }
}

/**
  * @brief          "runner_act" valiable initialization, include pid initialization, remote control data point initialization, runner motor
  *                 data point initialization.
  * @param[out]     runner_act_init: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"runner_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     runner_act_init:"runner_act"???????.
  * @retval         none
  */
static void mode_init(garbage_mode_t *garbage_mode_init)
{
	  if (garbage_mode_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		garbage_mode_init->last_garbage_mode = garbage_mode_init->garbage_mode = MODE_FREE;
		
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void mode_set(garbage_mode_t *garbage_mode_set)
{
    if (garbage_mode_set == NULL)
    {
        return;
    }

		/*三个超声传感器均被遮挡，持续2s，进入工作模式*/		
		if(mode_set_flag==1)
		{
			vTaskDelay(1000);		
			if(mode_set_flag==1)
					garbage_mode_set->garbage_mode = MODE_WORK;
		}

		/*按一下KEY键，退出工作模式*/	
		if(garbage_mode_set->last_garbage_mode == MODE_WORK && HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
		{
			vTaskDelay(20);
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
				garbage_mode_set->garbage_mode = MODE_FREE;
		}
		
		//工作模式下设置旋转方位
		if(garbage_mode_set->garbage_mode == MODE_WORK)
		{
		if(distance1<10.0f)
		{
			vTaskDelay(1000);			
			if(distance1<10.0f)
			{
				turn_set_flag=1;
				vTaskDelay(8000);
				turn_set_flag=0;
			}
		}		
		else if(distance2<10.0f)
		{
			vTaskDelay(1000);			
			if(distance2<10.0f)
			{
				turn_set_flag=2;
				//5秒后复原
				vTaskDelay(8000);
				turn_set_flag=0;				
			}
		}
		else if(distance3<10.0f)
		{
			vTaskDelay(1000);			
			if(distance3<10.0f)
			{
				turn_set_flag=3;
				//5秒后复原
				vTaskDelay(8000);				
				turn_set_flag=0;
			}				
		}
		else 
			{turn_set_flag=0;}
		}
		
}





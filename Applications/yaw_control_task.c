/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       yaw_control_task.c/h
  * @brief      Yaw轴控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-12-2024   Ignis             1. done
  *	 V1.0.1			April-20-2024   Ignis             1. 修改锁死模式下的控制方式为单位置环，优化移动模式下的pid
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "yaw_control_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "mode_set_task.h"


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
static void yaw_init(yaw_act_t *yaw_act_init);

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
static void yaw_set_mode(yaw_act_t *yaw_act_mode);
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
static void yaw_feedback_update(yaw_act_t *yaw_act_init);
/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          横移机构模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void yaw_mode_change_control_transit(yaw_act_t *yaw_act_transit);

/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */
static void yaw_control_loop(yaw_act_t *yaw_act_control);
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"gimbal_control"??????????pid??'???? ?????????'??????????????'????????????????'??
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void yaw_PID_init(yaw_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 yaw_PID_calc(yaw_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

yaw_act_t yaw_act;


/*外部引用一系列变量*/

extern motor_measure_t motor_data[9];//引用电机数据
extern garbage_mode_t garbage_mode;//引用工作模式


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
void yaw_control_task(void const * argument)
{
		//wait a time 
    vTaskDelay(YAW_TASK_INIT_TIME);
    //trans init
    yaw_init(&yaw_act);
    while(1)
    {
			yaw_set_mode(&yaw_act);                    //???????????g?
			yaw_feedback_update(&yaw_act);            //??????????
			yaw_mode_change_control_transit(&yaw_act);
      yaw_control_loop(&yaw_act);
			vTaskDelay(YAW_CONTROL_TIME_MS);
			yaw_act.last_yaw_mode = yaw_act.yaw_mode;
			

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
static void yaw_init(yaw_act_t *yaw_act_init)
{
	  if (yaw_act_init == NULL)
    {
        return;
    }

		
    //初始化横移机构模式
		yaw_act_init->last_yaw_mode = yaw_act_init->yaw_mode = YAW_FREE;
		//读取速度pid数值
		const static fp32 motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
		
		//初始化YAW电机指针
		yaw_act_init->motor_data.yaw_motor_measure = get_motor_measure_point(1, CAN_YAW_ID);


		
		//速度和角度pid初始化
		yaw_PID_init(&yaw_act_init->yaw_angle_pid, YAW_MOTOR_ANGLE_PID_MAX_OUT, YAW_MOTOR_ANGLE_PID_MAX_IOUT, YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD);
		PID_init(&yaw_act_init->yaw_speed_pid, PID_POSITION, motor_speed_pid, YAW_MOTOR_SPEED_PID_MAX_OUT, YAW_MOTOR_SPEED_PID_MAX_IOUT);
		
    yaw_feedback_update(yaw_act_init);
		
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
static void yaw_set_mode(yaw_act_t *yaw_act_mode)
{
    if (yaw_act_mode == NULL)
    {
        return;
    }
		//工作模式
		if(garbage_mode.garbage_mode == MODE_WORK)
		{
			yaw_act_mode->yaw_mode = YAW_LOCK;			
		}
			
		//自由模式
		else
			yaw_act_mode->yaw_mode = YAW_FREE;
}

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

static void yaw_feedback_update(yaw_act_t *yaw_act_update)
{
	if (yaw_act_update == NULL)
    {
        return;
    }

		yaw_act_update->motor_data.motor_ecd = yaw_act_update->motor_data.yaw_motor_measure->ecd;//更新电机ecd
		yaw_act_update->motor_data.motor_speed = yaw_act_update->motor_data.yaw_motor_measure->speed_rpm;
		//电机绝对角度
		yaw_act_update->motor_data.motor_angle = ECD2ANGLE_GIMBAL * (yaw_act_update->motor_data.yaw_motor_measure->ecd);

}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          横移机构模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void yaw_mode_change_control_transit(yaw_act_t *yaw_act_transit)
{
		if(yaw_act_transit == NULL)
		{
			return;
		}

		if(yaw_act_transit->yaw_mode == YAW_LOCK && (yaw_act_transit->last_yaw_mode == YAW_FREE))
		//状态由自由模式切换至锁死，记录当前电机绝对角度并设为目标值
		{
			yaw_act_transit->motor_data.motor_angle_set = yaw_act_transit->motor_data.motor_angle;
		}
		
}
	
/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */

static void yaw_control_loop(yaw_act_t *yaw_act_control)
{
	//电机速度
		static fp32 motor_speed = 0;
	
	/*自由状态，发送电流值为0*/
		if (yaw_act_control->yaw_mode == YAW_FREE)
		{

			yaw_act_control->motor_data.give_current = 0;			
		}
			
		/*锁死状态，使用角度环单环控制电机锁紧在当前绝对角度下*/		
		else if(yaw_act_control->yaw_mode == YAW_LOCK)
		{

				yaw_act_control->motor_data.give_current = yaw_PID_calc(&yaw_act_control->yaw_angle_pid, 
																																				yaw_act_control->motor_data.motor_angle, 
																																				yaw_act_control->motor_data.motor_angle_set,
																																				yaw_act_control->motor_data.motor_speed);				
			
		}
		CAN_cmd_yaw(yaw_act_control->motor_data.give_current);

}


/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */
uint8_t get_yaw_mode(void)
{
	return yaw_act.yaw_mode;
}
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"gimbal_control"??????????pid??'???? ?????????'??????????????'????????????????'??
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void yaw_PID_init(yaw_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}


static fp32 yaw_PID_calc(yaw_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}


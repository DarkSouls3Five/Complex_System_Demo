/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       translate_task.c/h
  * @brief      ºáÒÆ»ú¹¹ÈÎÎñ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-12-2024   Ignis             1. done
  *	 V1.0.1			April-20-2024   Ignis             1. ĞŞ¸ÄËøËÀÄ£Ê½ÏÂµÄ¿ØÖÆ·½Ê½Îªµ¥Î»ÖÃ»·£¬ÓÅ»¯ÒÆ¶¯Ä£Ê½ÏÂµÄpid
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "translate_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "mode_set_task.h"


double fabs(double a)
{
	if (a >= 0)
		return a;
	else 
		return -a;
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
static void trans_init(trans_act_t *trans_act_init);

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
static void trans_set_mode(trans_act_t *trans_act_mode);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????£????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */
static void trans_feedback_update(trans_act_t *trans_act_init);
/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ºáÒÆ»ú¹¹Ä£Ê½¸Ä±ä£¬ÓĞĞ©²ÎÊıĞèÒª¸Ä±ä£¬ÀıÈç¿ØÖÆyaw½Ç¶ÈÉè¶¨ÖµÓ¦¸Ã±ä³Éµ±Ç°yaw½Ç¶È
  * @param[out]     mode_change:"gimbal_control"±äÁ¿Ö¸Õë.
  * @retval         none
  */
static void trans_mode_change_control_transit(trans_act_t *trans_act_transit);

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
static void trans_control_loop(trans_act_t *trans_act_control);
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
static void trans_PID_init(trans_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 trans_PID_calc(trans_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

trans_act_t trans_act;
float trans_distance[2];//¼ÇÂ¼µç»ú×´Ì¬ÇĞ»»Ç°µÄ¾ø¶Ô½Ç¶È
float trans_distance1,trans_distance2;//debug¿´µÄ£¬Ã»É¶ÓÃ
uint8_t trans_motor_number;//½«ÒªÖ´ĞĞ¶¯×÷µÄµç»úĞòºÅ
uint8_t pos_A,pos_B,pos_C,pos_D;//¼ÇÂ¼¸÷ÇøÓòËù´¦µ²Î»

/*Íâ²¿ÒıÓÃÒ»ÏµÁĞ±äÁ¿*/
extern void init_ecd_record(motor_measure_t *motor_2006);//µç»ú¾ø¶Ô½Ç¶È³õÊ¼»¯
extern motor_measure_t motor_data[9];//ÒıÓÃµç»úÊı¾İ
extern garbage_mode_t garbage_mode;//ÒıÓÃ¹¤×÷Ä£Ê½
extern uint8_t infrared_return;//ÒıÓÃºìÍâ´«¸ĞÆ÷Êı¾İ


/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ºáÒÆ»ú¹¹ÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void translate_task(void const * argument)
{
		//wait a time 
    vTaskDelay(TRANS_TASK_INIT_TIME);
    //trans init
    trans_init(&trans_act);
    while(1)
    {
			trans_set_mode(&trans_act);                    //???????????g?
			trans_feedback_update(&trans_act);            //??????????
			trans_mode_change_control_transit(&trans_act);
      trans_control_loop(&trans_act);
			vTaskDelay(TRANS_CONTROL_TIME_MS);
			trans_act.last_trans_mode = trans_act.trans_mode;
			

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
static void trans_init(trans_act_t *trans_act_init)
{
	  if (trans_act_init == NULL)
    {
        return;
    }

		//³õÊ¼»¯µç»ú±àºÅÓë¸÷ÇøÓòµ²Î»
		trans_motor_number = 0;
		pos_A = pos_B = pos_C = pos_D = 0;
		
    //³õÊ¼»¯ºáÒÆ»ú¹¹Ä£Ê½
		trans_act_init->last_trans_mode = trans_act_init->trans_mode = TRANS_FREE;
		//¶ÁÈ¡ËÙ¶ÈpidÊıÖµ
		const static fp32 motor_speed_pid[3] = {TRANS_MOTOR_SPEED_PID_KP, TRANS_MOTOR_SPEED_PID_KI, TRANS_MOTOR_SPEED_PID_KD};
		
		//³õÊ¼»¯Èı¸öºáÒÆµç»úÖ¸Õë
		trans_act_init->motor_data[0].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS1_ID);
		trans_act_init->motor_data[1].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS2_ID);
		trans_act_init->motor_data[2].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS3_ID);

		//¼ÇÂ¼Èı¸öºáÒÆµç»ú³õÊ¼ecd
		init_ecd_record(&motor_data[0]);
		init_ecd_record(&motor_data[1]);		
		init_ecd_record(&motor_data[2]);	
		
		//ËÙ¶ÈºÍ½Ç¶Èpid³õÊ¼»¯
		trans_PID_init(&trans_act_init->trans_angle_pid, TRANS_MOTOR_ANGLE_PID_MAX_OUT, TRANS_MOTOR_ANGLE_PID_MAX_IOUT, TRANS_MOTOR_ANGLE_PID_KP, TRANS_MOTOR_ANGLE_PID_KI, TRANS_MOTOR_ANGLE_PID_KD);
		PID_init(&trans_act_init->trans_speed_pid, PID_POSITION, motor_speed_pid, TRANS_MOTOR_SPEED_PID_MAX_OUT, TRANS_MOTOR_SPEED_PID_MAX_IOUT);
		
    trans_feedback_update(trans_act_init);
		
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
static void trans_set_mode(trans_act_t *trans_act_mode)
{
    if (trans_act_mode == NULL)
    {
        return;
    }
		//¹¤×÷Ä£Ê½
		if(garbage_mode.garbage_mode == MODE_WORK)
		{
			if(infrared_return == 1)
			//ÇøÓò1×°Âú
			{
				trans_act_mode->trans_mode = TRANS_MOVE_L;
				trans_motor_number = 0;//µç»ú1Ö´ĞĞ¶¯×÷
			}
			
			else if(infrared_return == 2)
			//ÇøÓò2×°Âú
			{
				trans_act_mode->trans_mode = TRANS_MOVE_L;				
				trans_motor_number = 1;//µç»ú2Ö´ĞĞ¶¯×÷
			}
			else
				trans_act_mode->trans_mode = TRANS_LOCK;			
		}
			
		//×ÔÓÉÄ£Ê½
		else
		{
			trans_act_mode->trans_mode = TRANS_FREE;
			pos_A = pos_B = pos_C = pos_D = 0;		//·Ç¹¤×÷Ä£Ê½ÏÂ£¬½«µ²Î»È«¶¼³õÊ¼»¯Îª0
		}
}

/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????£????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */

static void trans_feedback_update(trans_act_t *trans_act_update)
{
	if (trans_act_update == NULL)
    {
        return;
    }
		//ºáÒÆµç»ú1
		trans_act_update->motor_data[0].motor_ecd = trans_act_update->motor_data[0].trans_motor_measure->ecd;//¸üĞÂµç»úecd
		trans_act_update->motor_data[0].motor_speed = trans_act_update->motor_data[0].trans_motor_measure->speed_rpm;
		//ºáÒÆµç»ú2
		trans_act_update->motor_data[1].motor_ecd = trans_act_update->motor_data[1].trans_motor_measure->ecd;//¸üĞÂµç»úecd
		trans_act_update->motor_data[1].motor_speed = trans_act_update->motor_data[1].trans_motor_measure->speed_rpm;
		//ºáÒÆµç»ú3
		trans_act_update->motor_data[2].motor_ecd = trans_act_update->motor_data[2].trans_motor_measure->ecd;//¸üĞÂµç»úecd
		trans_act_update->motor_data[2].motor_speed = trans_act_update->motor_data[2].trans_motor_measure->speed_rpm;

}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ºáÒÆ»ú¹¹Ä£Ê½¸Ä±ä£¬ÓĞĞ©²ÎÊıĞèÒª¸Ä±ä£¬ÀıÈç¿ØÖÆyaw½Ç¶ÈÉè¶¨ÖµÓ¦¸Ã±ä³Éµ±Ç°yaw½Ç¶È
  * @param[out]     mode_change:"gimbal_control"±äÁ¿Ö¸Õë.
  * @retval         none
  */
static void trans_mode_change_control_transit(trans_act_t *trans_act_transit)
{
		if(trans_act_transit == NULL)
		{
			return;
		}

		if(trans_act_transit->trans_mode == TRANS_LOCK && (trans_act_transit->last_trans_mode == TRANS_FREE))
		//×´Ì¬ÓÉ×ÔÓÉÄ£Ê½ÇĞ»»ÖÁËøËÀ£¬¼ÇÂ¼µ±Ç°µç»ú¾ø¶Ô½Ç¶È²¢ÉèÎªÄ¿±êÖµ
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_distance[i] = trans_act_transit->motor_data[i].trans_motor_measure->distance;
			}
		}
		if((trans_act_transit->trans_mode == TRANS_MOVE_L || trans_act_transit->trans_mode == TRANS_MOVE_R) 
			  && trans_act_transit->last_trans_mode == TRANS_LOCK)
		//×´Ì¬ÓÉËøËÀÇĞ»»ÖÁÒÆ¶¯Ä£Ê½£¬½«µ±Ç°¾ø¶Ô½Ç¶È¸³Öµ¸øtrans_distance
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_distance[i] = trans_act_transit->motor_data[i].trans_motor_measure->distance;
			}
		}
		
		if(trans_act_transit->trans_mode == TRANS_LOCK && 
			(trans_act_transit->last_trans_mode == TRANS_MOVE_L ||trans_act_transit->last_trans_mode == TRANS_MOVE_R))
		//×´Ì¬ÓÉÒÆ¶¯Ä£Ê½ÇĞ»»ÖÁËøËÀ£¬¼ÇÂ¼µ±Ç°µç»ú¾ø¶Ô½Ç¶È²¢ÉèÎªÄ¿±êÖµ
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_distance[i] = trans_act_transit->motor_data[i].trans_motor_measure->distance;
			}
		}
		trans_distance1=trans_distance[0];
		trans_distance2=trans_distance[1];
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

static void trans_control_loop(trans_act_t *trans_act_control)
{
	//µç»úËÙ¶È
		static fp32 motor_speed = 0;
	
	/*×ÔÓÉ×´Ì¬£¬·¢ËÍµçÁ÷ÖµÎª0*/
		if (trans_act_control->trans_mode == TRANS_FREE)
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_act_control->motor_data[i].give_current = 0;			
			}				
	}
				
		/*Ïò×óÒÆ¶¯×´Ì¬*/
		else if(trans_act_control->trans_mode == TRANS_MOVE_L)
		{
			motor_speed = TRANS_SET_SPEED;
			
			if(fabs(fabs(trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance - trans_distance[trans_motor_number]) - TRANS_MOVE_ANGLE) < TRANS_ERR)
			//ÒÆ¶¯¼´½«µ½Î»Ê±¸ÄÓÃÎ»ÖÃ»·
			{
				trans_act_control->motor_data[trans_motor_number].give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																											                trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance, 
																											                trans_distance[trans_motor_number] + TRANS_MOVE_ANGLE, 
																											                trans_act_control->motor_data[trans_motor_number].motor_speed);	
			}
			else
			//ÒÆ¶¯ÖĞÊ¹ÓÃµ¥ËÙ¶È»·			
			{
				trans_act_control->motor_data[trans_motor_number].motor_speed_set = motor_speed;
				trans_act_control->motor_data[trans_motor_number].give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																													trans_act_control->motor_data[trans_motor_number].motor_speed, trans_act_control->motor_data[trans_motor_number].motor_speed_set);			
			}
		}	


		/*ÏòÓÒÒÆ¶¯×´Ì¬*/
		else if(trans_act_control->trans_mode == TRANS_MOVE_R)
		{
			motor_speed = -TRANS_SET_SPEED;
			
			if(fabs(fabs(trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance - trans_distance[trans_motor_number]) + TRANS_MOVE_ANGLE) < TRANS_ERR)
			//ÒÆ¶¯¼´½«µ½Î»Ê±¸ÄÓÃÎ»ÖÃ»·
			{
				trans_act_control->motor_data[trans_motor_number].give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																											                trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance, 
																											                trans_distance[trans_motor_number] + TRANS_MOVE_ANGLE, 
																											                trans_act_control->motor_data[trans_motor_number].motor_speed);	
			}
			else
			{
			//ÒÆ¶¯ÖĞÊ¹ÓÃµ¥ËÙ¶È»·
				trans_act_control->motor_data[trans_motor_number].motor_speed_set = motor_speed;
				trans_act_control->motor_data[trans_motor_number].give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																													trans_act_control->motor_data[trans_motor_number].motor_speed, trans_act_control->motor_data[trans_motor_number].motor_speed_set);			
			}		
		}
			
		/*ËøËÀ×´Ì¬£¬Ê¹ÓÃ½Ç¶È»·µ¥»·¿ØÖÆµç»úËø½ôÔÚµ±Ç°¾ø¶Ô½Ç¶ÈÏÂ*/		
		else if(trans_act_control->trans_mode == TRANS_LOCK)
		{
			for (uint8_t i = 0; i < 2; i++)	
			{
				trans_act_control->motor_data[i].give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																																				trans_act_control->motor_data[i].trans_motor_measure->distance, 
																																				trans_distance[i], 
																																				trans_act_control->motor_data[i].motor_speed);				
			}
	}
		
		//¼ÆËãÍê±Ï£¬´ò°ü·¢ËÍËùÓĞµç»úµçÁ÷
		CAN_cmd_can1(trans_act_control->motor_data[0].give_current,trans_act_control->motor_data[1].give_current,trans_act_control->motor_data[2].give_current);
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
uint8_t get_trans_mode(void)
{
	return trans_act.trans_mode;
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
static void trans_PID_init(trans_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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


static fp32 trans_PID_calc(trans_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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





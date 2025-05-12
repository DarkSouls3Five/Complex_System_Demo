/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       translate_task.c/h
  * @brief      ���ƻ�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-12-2024   Ignis             1. done
  *	 V1.0.1			April-20-2024   Ignis             1. �޸�����ģʽ�µĿ��Ʒ�ʽΪ��λ�û����Ż��ƶ�ģʽ�µ�pid
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
  * @brief          ???_?????????�????????????y???????????????
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
  * @brief          ���ƻ���ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
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
float trans_distance[2];//��¼���״̬�л�ǰ�ľ��ԽǶ�
float trans_distance1,trans_distance2;//debug���ģ�ûɶ��
uint8_t trans_motor_number;//��Ҫִ�ж����ĵ�����
uint8_t pos_A,pos_B,pos_C,pos_D;//��¼������������λ

/*�ⲿ����һϵ�б���*/
extern void init_ecd_record(motor_measure_t *motor_2006);//������ԽǶȳ�ʼ��
extern motor_measure_t motor_data[9];//���õ������
extern garbage_mode_t garbage_mode;//���ù���ģʽ
extern uint8_t infrared_return;//���ú��⴫��������


/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ���ƻ�������
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

		//��ʼ���������������λ
		trans_motor_number = 0;
		pos_A = pos_B = pos_C = pos_D = 0;
		
    //��ʼ�����ƻ���ģʽ
		trans_act_init->last_trans_mode = trans_act_init->trans_mode = TRANS_FREE;
		//��ȡ�ٶ�pid��ֵ
		const static fp32 motor_speed_pid[3] = {TRANS_MOTOR_SPEED_PID_KP, TRANS_MOTOR_SPEED_PID_KI, TRANS_MOTOR_SPEED_PID_KD};
		
		//��ʼ���������Ƶ��ָ��
		trans_act_init->motor_data[0].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS1_ID);
		trans_act_init->motor_data[1].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS2_ID);
		trans_act_init->motor_data[2].trans_motor_measure = get_motor_measure_point(1, CAN_TRANS3_ID);

		//��¼�������Ƶ����ʼecd
		init_ecd_record(&motor_data[0]);
		init_ecd_record(&motor_data[1]);		
		init_ecd_record(&motor_data[2]);	
		
		//�ٶȺͽǶ�pid��ʼ��
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
		//����ģʽ
		if(garbage_mode.garbage_mode == MODE_WORK)
		{
			if(infrared_return == 1)
			//����1װ��
			{
				trans_act_mode->trans_mode = TRANS_MOVE_L;
				trans_motor_number = 0;//���1ִ�ж���
			}
			
			else if(infrared_return == 2)
			//����2װ��
			{
				trans_act_mode->trans_mode = TRANS_MOVE_L;				
				trans_motor_number = 1;//���2ִ�ж���
			}
			else
				trans_act_mode->trans_mode = TRANS_LOCK;			
		}
			
		//����ģʽ
		else
		{
			trans_act_mode->trans_mode = TRANS_FREE;
			pos_A = pos_B = pos_C = pos_D = 0;		//�ǹ���ģʽ�£�����λȫ����ʼ��Ϊ0
		}
}

/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????�????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */

static void trans_feedback_update(trans_act_t *trans_act_update)
{
	if (trans_act_update == NULL)
    {
        return;
    }
		//���Ƶ��1
		trans_act_update->motor_data[0].motor_ecd = trans_act_update->motor_data[0].trans_motor_measure->ecd;//���µ��ecd
		trans_act_update->motor_data[0].motor_speed = trans_act_update->motor_data[0].trans_motor_measure->speed_rpm;
		//���Ƶ��2
		trans_act_update->motor_data[1].motor_ecd = trans_act_update->motor_data[1].trans_motor_measure->ecd;//���µ��ecd
		trans_act_update->motor_data[1].motor_speed = trans_act_update->motor_data[1].trans_motor_measure->speed_rpm;
		//���Ƶ��3
		trans_act_update->motor_data[2].motor_ecd = trans_act_update->motor_data[2].trans_motor_measure->ecd;//���µ��ecd
		trans_act_update->motor_data[2].motor_speed = trans_act_update->motor_data[2].trans_motor_measure->speed_rpm;

}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���ƻ���ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void trans_mode_change_control_transit(trans_act_t *trans_act_transit)
{
		if(trans_act_transit == NULL)
		{
			return;
		}

		if(trans_act_transit->trans_mode == TRANS_LOCK && (trans_act_transit->last_trans_mode == TRANS_FREE))
		//״̬������ģʽ�л�����������¼��ǰ������ԽǶȲ���ΪĿ��ֵ
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_distance[i] = trans_act_transit->motor_data[i].trans_motor_measure->distance;
			}
		}
		if((trans_act_transit->trans_mode == TRANS_MOVE_L || trans_act_transit->trans_mode == TRANS_MOVE_R) 
			  && trans_act_transit->last_trans_mode == TRANS_LOCK)
		//״̬�������л����ƶ�ģʽ������ǰ���ԽǶȸ�ֵ��trans_distance
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_distance[i] = trans_act_transit->motor_data[i].trans_motor_measure->distance;
			}
		}
		
		if(trans_act_transit->trans_mode == TRANS_LOCK && 
			(trans_act_transit->last_trans_mode == TRANS_MOVE_L ||trans_act_transit->last_trans_mode == TRANS_MOVE_R))
		//״̬���ƶ�ģʽ�л�����������¼��ǰ������ԽǶȲ���ΪĿ��ֵ
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
	//����ٶ�
		static fp32 motor_speed = 0;
	
	/*����״̬�����͵���ֵΪ0*/
		if (trans_act_control->trans_mode == TRANS_FREE)
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				trans_act_control->motor_data[i].give_current = 0;			
			}				
	}
				
		/*�����ƶ�״̬*/
		else if(trans_act_control->trans_mode == TRANS_MOVE_L)
		{
			motor_speed = TRANS_SET_SPEED;
			
			if(fabs(fabs(trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance - trans_distance[trans_motor_number]) - TRANS_MOVE_ANGLE) < TRANS_ERR)
			//�ƶ�������λʱ����λ�û�
			{
				trans_act_control->motor_data[trans_motor_number].give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																											                trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance, 
																											                trans_distance[trans_motor_number] + TRANS_MOVE_ANGLE, 
																											                trans_act_control->motor_data[trans_motor_number].motor_speed);	
			}
			else
			//�ƶ���ʹ�õ��ٶȻ�			
			{
				trans_act_control->motor_data[trans_motor_number].motor_speed_set = motor_speed;
				trans_act_control->motor_data[trans_motor_number].give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																													trans_act_control->motor_data[trans_motor_number].motor_speed, trans_act_control->motor_data[trans_motor_number].motor_speed_set);			
			}
		}	


		/*�����ƶ�״̬*/
		else if(trans_act_control->trans_mode == TRANS_MOVE_R)
		{
			motor_speed = -TRANS_SET_SPEED;
			
			if(fabs(fabs(trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance - trans_distance[trans_motor_number]) + TRANS_MOVE_ANGLE) < TRANS_ERR)
			//�ƶ�������λʱ����λ�û�
			{
				trans_act_control->motor_data[trans_motor_number].give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																											                trans_act_control->motor_data[trans_motor_number].trans_motor_measure->distance, 
																											                trans_distance[trans_motor_number] + TRANS_MOVE_ANGLE, 
																											                trans_act_control->motor_data[trans_motor_number].motor_speed);	
			}
			else
			{
			//�ƶ���ʹ�õ��ٶȻ�
				trans_act_control->motor_data[trans_motor_number].motor_speed_set = motor_speed;
				trans_act_control->motor_data[trans_motor_number].give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																													trans_act_control->motor_data[trans_motor_number].motor_speed, trans_act_control->motor_data[trans_motor_number].motor_speed_set);			
			}		
		}
			
		/*����״̬��ʹ�ýǶȻ��������Ƶ�������ڵ�ǰ���ԽǶ���*/		
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
		
		//������ϣ�����������е������
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





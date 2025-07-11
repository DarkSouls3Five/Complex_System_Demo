#ifndef TRANSLATE_TASK_H
#define TRANSLATE_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define TRANS_TASK_INIT_TIME 300
#define TRANS_CONTROL_TIME_MS 5
#define PI					3.14159265358979f
#define TRANS_MOVE_ANGLE 500.0f//移动二分之一边长
#define TRANS_MOVE_ANGLE_HALF 250.0f//移动四分之一边长
#define TRANS_DISTANCE_ERR 20.0f//位移偏差
#define TRANS_ERR 10.0f
#define DIR_L 1
#define DIR_R 0

#define TRANS_MOTOR_SPEED_PID_KP 3.0f
#define TRANS_MOTOR_SPEED_PID_KI 0.0f
#define TRANS_MOTOR_SPEED_PID_KD -0.01f

#define TRANS_MOTOR_SPEED_PID_MAX_OUT 800.f
#define TRANS_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define TRANS_MOTOR_ANGLE_PID_KP 0.60f
#define TRANS_MOTOR_ANGLE_PID_KI 0.0f
#define TRANS_MOTOR_ANGLE_PID_KD 0.0f

#define TRANS_MOTOR_ANGLE_PID_MAX_OUT 1000.0f
#define TRANS_MOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define TRANS_SET_SPEED 600.0f

typedef enum
{
  TRANS_FREE,
	TRANS_LOCK,
	TRANS_MOVE_L,//单电机左移
	TRANS_MOVE_R,//单电机右移

} trans_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 last_err;
	
    fp32 max_out;
    fp32 max_iout;
		fp32 max_dout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
		fp32 dT;
} trans_PID_t;

typedef struct
{
  const motor_measure_t *trans_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
	fp32 motor_ecd;     //rad
  fp32 motor_ecd_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
  int16_t give_current;
} trans_motor_t;

typedef struct
{
//  const RC_ctrl_t *RC_data;               //????'?�?????????, the point to remote control
  trans_mode_e last_trans_mode;               //state machine. ???????????
	trans_mode_e trans_mode; 
  trans_motor_t motor_data[3];          //chassis motor data.??????????
	trans_PID_t trans_angle_pid;
  pid_type_def trans_speed_pid;             //motor speed PID.?????????pid


} trans_act_t;


/**
  * @brief          runner task, osDelay RUNNER_CONTROL_TIME_MS (5ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ????,?? RUNNER_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ?
  * @retval         none
  */
extern void trans_task(void const * argument);
extern uint8_t get_trans_mode(void);
extern trans_act_t trans_act;
#endif

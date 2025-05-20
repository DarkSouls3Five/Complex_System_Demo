#ifndef YAW_CONTROL_TASK_H
#define YAW_CONTROL_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define YAW_TASK_INIT_TIME 300
#define YAW_CONTROL_TIME_MS 5
#define PI					3.14159265358979f

#define YAW_MOTOR_SPEED_PID_KP 5.0f
#define YAW_MOTOR_SPEED_PID_KI 0.0f
#define YAW_MOTOR_SPEED_PID_KD -0.05f

#define YAW_MOTOR_SPEED_PID_MAX_OUT 6000.f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define YAW_MOTOR_ANGLE_PID_KP 800.0f
#define YAW_MOTOR_ANGLE_PID_KI 150.0f
#define YAW_MOTOR_ANGLE_PID_KD -700.0f

#define YAW_MOTOR_ANGLE_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define YAW_SET_SPEED 60.0f

//µç»ú½Ç¶È
#define YAW_ANGEL_INIT 120.0f
#define YAW_ANGEL_LEFT 30.0f
#define YAW_ANGEL_RIGHT 210.0f


typedef enum
{
  YAW_FREE,
	YAW_LOCK,

	
} yaw_mode_e;

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
} yaw_PID_t;

typedef struct
{
  const motor_measure_t *yaw_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
	fp32 motor_ecd;     //rad
  fp32 motor_ecd_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
  int16_t give_current;
} yaw_motor_t;

typedef struct
{
//  const RC_ctrl_t *RC_data;               //????'??????????, the point to remote control
  yaw_mode_e last_yaw_mode;               //state machine. ???????????
	yaw_mode_e yaw_mode; 
  yaw_motor_t motor_data;          //chassis motor data.??????????
	yaw_PID_t yaw_angle_pid;
  pid_type_def yaw_speed_pid;             //motor speed PID.?????????pid


} yaw_act_t;


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
extern uint8_t get_yaw_mode(void);
#endif

#ifndef MODE_SET_TASK_H
#define MODE_SET_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define MODE_SET_TASK_INIT_TIME 300
#define MODE_SET_TIME_MS 5

typedef enum
{
  MODE_FREE,	//自由模式
	MODE_WORK,	//工作模式
	
} garbage_mode_e;


typedef struct
{
  const RC_ctrl_t *RC_data;               //????'??????????, the point to remote control
  garbage_mode_e last_garbage_mode;               //state machine. ???????????
	garbage_mode_e garbage_mode; 

} garbage_mode_t;



#endif

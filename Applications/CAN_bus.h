/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_bus.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "struct_typedef.h"

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8192
#define ECD2ANGLE 2 * 3.1415926535f / ECD_RANGE
#define REDUCTION_GIMBAL 1.0f		//云台减速比
#define ECD2ANGLE_GIMBAL  REDUCTION_GIMBAL* 360.0f / ECD_RANGE
#define R_WHEEL 2.5f //带轮直径，单位cm
#define ANGLE2LENGTH R_WHEEL*3.1415926535f/360f//电机出轴角度转换到移动距离


/* GPIO send and receive ID */
typedef enum
{
	  CAN_TX_YAW_ID = 0X1FF,
		CAN_TX_TRANS_ID = 0X200,//三个横移电机
    CAN_TRANS1_ID = 0x201,	
		CAN_TRANS2_ID = 0x202,
	  CAN_TRANS3_ID = 0x203,
		CAN_YAW_ID = 0x208

} can1_msg_id_e;

typedef enum
{
	  CAN_TX_LEFT_ID = 0X200,
		CAN_TX_RIGHT_ID = 0X1FF,
    CAN_3508_FRICL1_ID = 0x201,
    CAN_3508_FRICL2_ID = 0x202,
		CAN_3508_FRICL3_ID = 0x203,
		CAN_3508_FRICR1_ID = 0x205,
		CAN_3508_FRICR2_ID = 0x206,
    CAN_3508_FRICR3_ID = 0x207,

} can2_msg_id_e;

//rm motor data
typedef struct
{

	uint16_t ecd;
	uint16_t init_ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	uint16_t last_ecd;
	int16_t ecd_count;
	int16_t round;
	fp32 distance;
	
} motor_measure_t;



extern void get_motor_measure(motor_measure_t *ptr, uint8_t *data);  
extern void CAN_cmd_can1(int16_t motor1, int16_t motor2, int16_t motor3);
extern void CAN_cmd_yaw(int16_t motor4);
extern void CAN_cmd_can2(int16_t motor1, int16_t motor2, int16_t motor3);
extern const motor_measure_t *get_motor_measure_point(uint8_t bus, uint16_t id); 



#endif

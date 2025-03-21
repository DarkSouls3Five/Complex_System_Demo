/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_bus.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
#define REDUCTION_GIMBAL 0.2871287f		//��̨���ٱ�
#define ECD2ANGLE_GIMBAL  REDUCTION_GIMBAL* 360.0f / ECD_RANGE


/* GPIO send and receive ID */
typedef enum
{
	  CAN_TX_REAR_ID = 0X1FF,
		CAN_TX_TRANS_ID = 0X200,//�������Ƶ��
    CAN_TRANS1_ID = 0x201,	
		CAN_TRANS2_ID = 0x202,
	  CAN_TRANS3_ID = 0x203,

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
extern void CAN_cmd_can2(int16_t motor1, int16_t motor2, int16_t motor3);
extern const motor_measure_t *get_motor_measure_point(uint8_t bus, uint16_t id); 



#endif

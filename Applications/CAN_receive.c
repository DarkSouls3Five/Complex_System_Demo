#include "CAN_receive.h"
#include "main.h"
#include "MotorTask.h"

extern CAN_HandleTypeDef hcan1;

//电机数据获取
void get_motor_measure(motor_measure_t* motor_state,uint8_t rx_data[8])
{
	(motor_state)->last_ecd = (motor_state)->ecd;                                   
  (motor_state)->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);            
  (motor_state)->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);      
  (motor_state)->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);  
  (motor_state)->temperate = (rx_data)[6]; 
}
		

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan==&hcan1)
	{
		switch (rx_header.StdId)
		{
		
			case 0x201:
			{
				get_motor_measure(&motor1,rx_data);
				if((motor1.ecd - motor1.last_ecd) > 4096)
				{
					motor_3508.round_num --;
				}
				else if((motor1.ecd - motor1.last_ecd) < -4096)
				{
					motor_3508.round_num ++;
				}
				motor_3508.relative_angle = rad_format((float)((motor1.ecd - motor_3508.offecd_ecd) + motor_3508.round_num * 8092) / 8192 / 19.02f * 2 * PI);
				motor_3508.relative_speed = (float)motor1.speed_rpm / 8192 * 2 * PI / 19.02f;
				break;
			}
			case 0x205:
			{
				get_motor_measure(&motor2,rx_data);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}


/**
  * @brief          CAN发送函数，发送电机转矩电流
  * @param[in]      id为0x201~0x204各个电机的转矩电流值
  * @retval         none
  */
void CAN_cmd_motor3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x200;
	Txheader2.IDE = CAN_ID_STD;
	Txheader2.DLC = 0x08;
	Txheader2.RTR = CAN_RTR_DATA;
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor1 >> 8 ;
	CAN_send_data2[1] = motor1;
	CAN_send_data2[2] = motor2 >> 8 ;
	CAN_send_data2[3] = motor2;
	CAN_send_data2[4] = motor3 >> 8 ;
	CAN_send_data2[5] = motor3;
	CAN_send_data2[6] = motor4 >> 8 ;
	CAN_send_data2[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

void CAN_cmd_motor6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x1ff;
	Txheader2.IDE = CAN_ID_STD;
	Txheader2.DLC = 0x08;
	Txheader2.RTR = CAN_RTR_DATA;
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor1 >> 8 ;
	CAN_send_data2[1] = motor1;
	CAN_send_data2[2] = motor2 >> 8 ;
	CAN_send_data2[3] = motor2;
	CAN_send_data2[4] = motor3 >> 8 ;
	CAN_send_data2[5] = motor3;
	CAN_send_data2[6] = motor4 >> 8 ;
	CAN_send_data2[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}


#include "remote_control.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"


extern uint8_t sbus_buf[18];
Rc_ctrl ctl;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	  ctl.rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    ctl.rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    ctl.rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    ctl.rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    ctl.rc.s1 = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    ctl.rc.s2 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	
}

#ifndef _USART_TASK_H_
#define _USART_TASK_H_
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

//阻塞式发送串口数据函数
uint8_t ESP8266_SendString(const char *str);

//初始化wifi模块
void init_ESP8266(void);

//UDP发送数据函数
uint8_t ESP8266_UDP_Send  (const char *data);

//DMA发送
void usart_printf(const char *fmt,...); 

#endif

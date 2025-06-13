#ifndef _USART_TASK_H_
#define _USART_TASK_H_
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

//����ʽ���ʹ������ݺ���
uint8_t ESP8266_SendString(const char *str);

//��ʼ��wifiģ��
void init_ESP8266(void);

//UDP�������ݺ���
uint8_t ESP8266_UDP_Send  (const char *data);

//DMA����
void usart_printf(const char *fmt,...); 

#endif

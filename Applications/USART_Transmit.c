#include "USART_Transmit.h"
#include "stdarg.h"
#include "bsp_usart.h"


extern float distance1,distance2,distance3;
uint8_t usart_c = 0;
char buffer[128];

//��������
void USART_Transmit(void const * argument)
{
		osDelay(1000);
		init_ESP8266();

		while(1)
		{

			int len = snprintf(buffer, sizeof(buffer), "D1:%.2f,D2:%.2f,D3:%.2f", distance1, distance2, distance3);			
			ESP8266_SendString(buffer);
			osDelay(1000); // �������ݷ���Ƶ�ʣ�1�뷢��1��		

		}
	
}



//��ʼ��
void init_ESP8266(void)
{
    ESP8266_SendString("AT");
    HAL_Delay(500);
    
    ESP8266_SendString("AT+CWMODE=1");  // ����ΪSTAģʽ
    HAL_Delay(500);
    
    ESP8266_SendString("AT+CWJAP=\"TJSP\",\"06010602\""); // ����WiFi
    HAL_Delay(3000);
	
	  ESP8266_SendString("AT+ATKCLDSTA=\"22495854243484409207\",\"12345678\""); // ����ԭ���Ʒ�����
    HAL_Delay(2000);
}


//���ڷ�������
uint8_t ESP8266_SendString(const char *str)
{
    HAL_StatusTypeDef status;
    
    // �����������ݣ�500ms��ʱ��
    status = HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
    if(status != HAL_OK) {
        return 0; // ����ʧ��
    }
    
    // ���ͻ��з���100ms��ʱ��
    status = HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
    return (status == HAL_OK) ? 1 : 0;
}



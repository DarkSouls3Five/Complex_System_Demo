#include "USART_Transmit.h"
#include "stdarg.h"
#include "bsp_usart.h"
#include "translate_task.h" 

extern float distance1,distance2,distance3;
extern trans_act_t trans_act;
extern uint8_t infrared_return;
uint8_t usart_c = 0;
char buffer[128];

//????????
void USART_Transmit(void const * argument)
{
		osDelay(1000);
		init_ESP8266();

		while(1)
		{

			// int len = snprintf(buffer, sizeof(buffer), "D1:%.2f,D2:%.2f,D3:%.2f", distance1, distance2, distance3);			
			// ESP8266_SendString(buffer);
			// osDelay(1000); // ???????????????1????1??		
            float m0 = trans_act.motor_data[0].trans_motor_measure->distance;
            float m1 = trans_act.motor_data[1].trans_motor_measure->distance;
            float m2 = trans_act.motor_data[2].trans_motor_measure->distance;
            // 将 3 路距离 + 红外返回值 格式化到 buffer
            int n = snprintf(buffer, sizeof(buffer),
                    "D1:%.2f,D2:%.2f,D3:%.2f,IR:%u,"
                    "M0:%.2f,M1:%.2f,M2:%.2f",
                    distance1, distance2, distance3,
                    infrared_return,
                    m0, m1, m2);
            if (n < 0 || n >= sizeof(buffer)) {
                // 格式化失败或截断，直接延时再重来
                osDelay(1000);
                continue;
            }

            // 通过 UDP 发出
            if (!ESP8266_UDP_Send(buffer)) {
                // 发送失败，可选择重连或重试
                // init_ESP8266();
            }


                // 3) 下次再发前等待 1s
                osDelay(1000);

		}
	
}



//?????
void init_ESP8266(void)
{
    ESP8266_SendString("AT");
    HAL_Delay(500);
    
    ESP8266_SendString("AT+CWMODE=1");  // ?????STA??
    HAL_Delay(500);
    
    ESP8266_SendString("AT+CWJAP=\"TJSP\",\"06010602\""); // ????WiFi
//    ESP8266_SendString("AT+CWJAP=\"407\",\"wangyafei666\""); // ????WiFi
    HAL_Delay(3000);

    ESP8266_SendString("AT+CIPMUX=0"); //UDP单连接模式
    HAL_Delay(500);

    // 建立 UDP 连接
    // 设置实际的远端IP，现假设为 IP 是 192.168.1.100，端口 8088，本地端口不指定，IP需要根据实际情况修改
    ESP8266_SendString("AT+CIPSTART=\"UDP\",\"192.168.43.79\",8088");
    HAL_Delay(1000);
	
	// ESP8266_SendString("AT+ATKCLDSTA=\"22495854243484409207\",\"12345678\""); // ??????????????
    // HAL_Delay(2000);
}


//???????????
uint8_t ESP8266_SendString(const char *str)
{
    HAL_StatusTypeDef status;
    
    // ?????????????500ms?????
    status = HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
    if(status != HAL_OK) {
        return 0; // ???????
    }
    
    // ??????з???100ms?????
    status = HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
    return (status == HAL_OK) ? 1 : 0;
}

uint8_t ESP8266_UDP_Send(const char *data)
{
    char cmd[32];
    int  len = strlen(data);

    // 1) 先发送 CIPSEND 告知长度
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    if (!ESP8266_SendString(cmd)) return 0;
    osDelay(200);               // 等 “>” 提示

    // 2) 再发真正的数据
    if (!ESP8266_SendString(data)) return 0;
    osDelay(200);               // 等发送完成

    return 1;
}

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB灯效。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "led_flow_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"
#include "mode_set_task.h"


#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_ALPHA_CHANGE_TIME  100
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭) -> 蓝 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
extern garbage_mode_t garbage_mode;

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void led_RGB_flow_task(void const * argument)
{
    uint16_t i, j;
    fp32 delta_alpha, delta_red, delta_green, delta_blue;
    fp32 alpha,red,green,blue;
    uint32_t aRGB;

    while(1)
    {

       if(garbage_mode.garbage_mode==MODE_WORK)
			 {
         alpha = (RGB_flow_color[1] & 0xFF000000) >> 24;
         red = ((RGB_flow_color[1] & 0x00FF0000) >> 16);
         green = ((RGB_flow_color[1] & 0x0000FF00) >> 8);
         blue = ((RGB_flow_color[1] & 0x000000FF) >> 0);
				 
				 delta_alpha = (fp32)((RGB_flow_color[1 + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[1] & 0xFF000000) >> 24);
				 delta_red = (fp32)((RGB_flow_color[1 + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[1] & 0x00FF0000) >> 16);
         delta_green = (fp32)((RGB_flow_color[1 + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[1] & 0x0000FF00) >> 8);
         delta_blue = (fp32)((RGB_flow_color[1 + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[1] & 0x000000FF) >> 0);
	
				 delta_alpha /= RGB_FLOW_ALPHA_CHANGE_TIME;
				 delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
         delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
         delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
			
				 for(j = 0; j < RGB_FLOW_ALPHA_CHANGE_TIME; j++)
         {
						alpha += delta_alpha;
					 
						aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
						aRGB_led_show(aRGB);
						osDelay(1);		
				 }
			 }
			else
			{
				for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
        {
					if(garbage_mode.garbage_mode==MODE_WORK)
					//一旦进入工作模式立刻停止流水灯循环，切换为绿灯闪烁指示
					{
						continue;
					}
						alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
            red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
            green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
            blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
            delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
            delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
            delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
							if(garbage_mode.garbage_mode==MODE_WORK)
							//一旦进入工作模式立刻停止流水灯循环，切换为绿灯闪烁指示
							{
								continue;
							}
								alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                aRGB_led_show(aRGB);
                osDelay(1);
            }
        }
			}
    }
}



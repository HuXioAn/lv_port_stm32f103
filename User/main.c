
#include "main.h"
#include "stm32f1xx.h"
#include "./usart/bsp_debug_usart.h"
#include <stdio.h>
#include "./lcd/bsp_ili9341_lcd.h"
#include "lvgl.h"

#define LCD_VERTICAL_RES 240
#define LCD_HORIZONTAL_RES 320

#define BUFFER_WIDTH 24

static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t      disp_drv;

static lv_color_t buf_1[LCD_HORIZONTAL_RES * BUFFER_WIDTH];
static lv_color_t buf_2[LCD_HORIZONTAL_RES * BUFFER_WIDTH];


int main(void)
{   
  /* 设定系统时钟为72MHz */
  SystemClock_Config();	
	
	/* 配置串口1为：115200 8-N-1 */
	DEBUG_USART_Config();
  
  lv_init();
  ILI9341_Init ();         //LCD 初始化

  //初始化buffer
	lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2, LCD_HORIZONTAL_RES*BUFFER_WIDTH);
  //初始化显示驱动
  lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
  disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
  disp_drv.hor_res = LCD_HORIZONTAL_RES;                 /*Set the horizontal resolution in pixels*/
  disp_drv.ver_res = LCD_VERTICAL_RES;                 /*Set the vertical resolution in pixels*/

  lv_disp_t * disp;
  disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/



}




static void Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}


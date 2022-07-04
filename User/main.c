
#include "main.h"
#include "stm32f1xx.h"
#include "./usart/bsp_debug_usart.h"
#include <stdio.h>
#include "./lcd/bsp_ili9341_lcd.h"
#include "lvgl.h"
#include "ILI9341.h"
#include "XPT2046.h"

#define LCD_VERTICAL_RES 320
#define LCD_HORIZONTAL_RES 240

#define BUFFER_WIDTH 10

static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;

static lv_color_t buf_1[LCD_HORIZONTAL_RES * BUFFER_WIDTH];
// static lv_color_t buf_2[LCD_HORIZONTAL_RES * BUFFER_WIDTH];

int main(void)
{
  //初始化HAL，使用systick的1ms节拍来为lvgl产生时钟
  HAL_Init();
  /* 设定系统时钟为72MHz */
  SystemClock_Config();

  /* 配置串口1为：115200 8-N-1 */
  DEBUG_USART_Config();

  lv_init();
  // ili9341_init();         //LCD 初始化

  ILI9341_Init();

  // xpt2046_init();
  //初始化buffer
  lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LCD_HORIZONTAL_RES * BUFFER_WIDTH);
  //初始化显示驱动
  lv_disp_drv_init(&disp_drv);           /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf;         /*Set an initialized buffer*/
  disp_drv.flush_cb = ili9341_flush;     /*Set a flush callback to draw to the display*/
  disp_drv.hor_res = LCD_HORIZONTAL_RES; /*Set the horizontal resolution in pixels*/
  disp_drv.ver_res = LCD_VERTICAL_RES;   /*Set the vertical resolution in pixels*/
                                         // disp_drv.rotated=LV_DISP_ROT_270;
  lv_disp_t *disp;
  disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

  lv_obj_t *obj1;
  obj1 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(obj1, 60, 60);
  // lv_obj_align(obj1, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_pos(obj1, 10, 10);

  lv_obj_t *label1 = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(label1, LV_LABEL_LONG_WRAP); /*Break the long lines*/
  lv_label_set_recolor(label1, true);                 /*Enable re-coloring by commands in the text*/
  lv_label_set_text(label1, "#0000ff Huxiaoan# #ff00ff 胡小安# #ff0000 Huxiaoan#  "
                            "#000000 Lvgl V8.2 on STM32F103@72MHz.#");
  lv_obj_set_style_text_font(label1, &lv_font_montserrat_20, 0);
  lv_obj_set_width(label1, 150); /*Set smaller width to make the lines wrap*/
  lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);

  lv_obj_t *label2 = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(label2, LV_LABEL_LONG_SCROLL_CIRCULAR); /*Circular scroll*/
  lv_obj_set_width(label2, 150);
  lv_label_set_text(label2, "Huxiaoan 胡小安 Huxiaoan ");
  lv_obj_set_style_text_font(label2, &lv_font_montserrat_20, 0);
  lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);

  // lvgl时钟调度
  while (1)
  {
    lv_timer_handler();
    HAL_Delay(5);
  }
}

static void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--)
    ;
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
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }
}

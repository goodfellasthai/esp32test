#pragma once
#include <wyconfig.h>

#include <lvgl.h>
#include <lv_conf.h>
//#include <demos/lv_demos.h>
#include "../libraries/lvgl/demos/lv_demos.h"
//#include <esp_heap_caps.h>
#include <Display_ST7789.h>

#define LVGL_WIDTH    (TFT_WIDTH)
#define LVGL_HEIGHT   TFT_HEIGHT
#define LVGL_BUF_LEN  (TFT_WIDTH * TFT_HEIGHT / 20)

#define LVGL_TICK_PERIOD_MS  5

void Lvgl_Init(void);
void Timer_Loop(void);

void Lvgl_print(const char * buf);
void Lvgl_Display_LCD( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ); // Displays LVGL content on the LCD.    This function implements associating LVGL data to the LCD screen
void Lvgl_Touchpad_Read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data );                // Read the touchpad
void example_increase_lvgl_tick(void *arg);

#pragma once
#include <Arduino.h>
#include <SPI.h>
#define LCD_WIDTH   172 //LCD width
#define LCD_HEIGHT  320 //LCD height

#define SPIFreq                        80000000
#define PIN_NUM_MISO           5
#define PIN_NUM_MOSI           6
#define PIN_NUM_SCLK           7
#define PIN_NUM_LCD_CS         14
#define PIN_NUM_LCD_DC         15
#define PIN_NUM_LCD_RST        21
#define PIN_NUM_BK_LIGHT       22
#define Frequency       1000      // PWM frequencyconst    ESP32S3测得最低允许153Hz
#define Resolution      10        // PWM分辨率, 取值范围是 1 到 16 ,分辨率的值越高，PWM的精细度越高，可以产生更多的不同占空比级别。

#define VERTICAL   0
#define HORIZONTAL 1

#define Offset_X 34
#define Offset_Y 0


void LCD_SetCursor(uint16_t x1, uint16_t y1, uint16_t x2,uint16_t y2);

void LCD_Init(void);
void LCD_SetCursor(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t  Yend);
void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,uint16_t* color);

void Backlight_Init(void);
void Set_Backlight(uint8_t Light);

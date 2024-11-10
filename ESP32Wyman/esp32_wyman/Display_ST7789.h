#pragma once
#include "wyconfig.h"

#include "Display_ST7789.h"
#include <SPI.h>

void LCD_Init(void);
void LCD_SetCursor(uint16_t x1, uint16_t y1, uint16_t x2,uint16_t y2);
void LCD_SetCursor(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t  Yend);
void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,uint16_t* color);

void Backlight_Init(void);
void Set_Backlight(uint8_t Light);

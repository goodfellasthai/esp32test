/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include "SD_Card.h"
#include "Display_ST7789.h"
#include "LCD_Image.h"

void setup()
{
  Flash_test();  
  LCD_Init();    
  SD_Init();   
  Display_Image("/",".png", 0);
  pinMode(BOOT_KEY_PIN, INPUT);   
  
}

void loop()
{
  Image_Next_Loop("/",".png",300);
  delay(5);
}

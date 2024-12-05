/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include "SD_Card.h"
#include "Display_ST7789.h"
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
void setup()
{       
  Flash_test();
  LCD_Init();
  Lvgl_Init();
  SD_Init();   

  Lvgl_Example1();     
  // lv_demo_widgets();               
  // lv_demo_benchmark();          
  // lv_demo_keypad_encoder();     
  // lv_demo_music();  
  // lv_demo_stress();   

  Wireless_Test2();  
}

void loop()
{
  Timer_Loop();
  delay(5);
}

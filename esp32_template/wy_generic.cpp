 #include "wy_generic.h"
 #if defined(WYMAN_LORA_GENERIC)
 #pragma once
// All libraries to be searched in the path in triangular quotes libraries in same directory double quotes
// libraries required to compile the .cpp should be included in the cpp only headers in the same directory to be included in the .h module file
// can't put the main .cpp reliant libraries in .h as would conflict with other libraries using the same functions
// the .h always needs called first in the .cpp module file so that the application knows the functions it can use for the board
#include <HT_SSD1306Wire.h>
SSD1306Wire  generic_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

void logo(){
	generic_display.clear();
	generic_display.drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
	generic_display.display();
}

// Allows power to the OLED
void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  
}
// Stops power to the OLED
void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void wy_generic_setup() {
	VextON();
	generic_display.init();
	// Show logo for 5 seconds
	logo();
	delay(5000);
	generic_display.clear();
  generic_display.display();
	// Delay for 2 seconds after clearing display for repeat
  delay(2000);

}

void wy_generic_loop() {
	logo();
	delay(5000);
	generic_display.clear();
  generic_display.display();
	// Delay for 2 seconds after clearing display for repeat
  delay(2000);


}

#endif
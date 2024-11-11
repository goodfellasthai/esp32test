#include "wyconfig.h"

void setup() {
    heltec_Init();
}

void loop() {
    heltec_loopCode();
    delay(10);
}

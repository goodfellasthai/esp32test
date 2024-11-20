#include "wy_cp2.h"
#if defined(WYMAN_M5STICKCP2)

#pragma once
// All libraries to be searched in the path in triangular quotes libraries in same directory double quotes
// libraries required to compile the .cpp should be included in the cpp only headers in the same directory to be included in the .h module file
// can't put the main .cpp reliant libraries in .h as would conflict with other libraries using the same functions
// the .h always needs called first in the .cpp module file so that the application knows the functions it can use for the board
#include <M5StickCPlus2.h>

void draw_function(LovyanGFX* gfx) {
    int x      = rand() % gfx->width();
    int y      = rand() % gfx->height();
    int r      = (gfx->width() >> 4) + 2;
    uint16_t c = rand();
    gfx->fillRect(x - r, y - r, r * 2, r * 2, c);
}

void wy_cp2_setup() {
    auto cfg = M5.config();
    StickCP2.begin(cfg);
    int textsize = StickCP2.Display.height() / 60;
    if (textsize == 0) {
        textsize = 1;
    }
    StickCP2.Display.setTextSize(textsize);
}

void wy_cp2_loop() {
    int x      = rand() % StickCP2.Display.width();
    int y      = rand() % StickCP2.Display.height();
    int r      = (StickCP2.Display.width() >> 4) + 2;
    uint16_t c = rand();
    StickCP2.Display.fillCircle(x, y, r, c);
    draw_function(&StickCP2.Display);
}

#endif
#pragma once
// All libraries to be searched in the path in triangular quotes libraries in same directory double quotes
// libraries required to compile the .cpp should be included in the cpp only headers in the same directory to be included in the .h module file
// can't put the main .cpp reliant libraries in .h as would conflict wutg libraries using the same functions with header called first which is required
// so the appication knows the functions it can call
#include "wyconfig.h"
#include "wy_v3_simple_images.h"

void wy_v3_simple_setup();
void wy_v3_simple_loop();

#pragma once
// All libraries to be searched in the path in triangular quotes libraries in same directory double quotes
// libraries required to compile the .cpp should be included in the cpp only headers in the same directory to be included in the .h module file
// can't put the main .cpp reliant libraries in .h as would conflict with other libraries using the same functions
// the .h always needs called first in the .cpp module file so that the application knows the functions it can use for the board
#include "wyconfig.h"

void wy_node_master_setup();
void wy_node_master_loop();
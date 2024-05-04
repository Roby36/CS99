
#pragma once

#include "set.h"

/* Main encapsulated functions exposed here for usage in other files */

void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols);
set_t * load_json(const char * file_name);





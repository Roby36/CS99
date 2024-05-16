
#pragma once

#include "utils.h"
#include "commonmacros.h"

/* Main encapsulated functions exposed here for usage in other files */
void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols);
Params * load_json(const char * file_name);


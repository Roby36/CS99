
#pragma once

#include "cJSON.h"
#include "utils.h"

/* Main encapsulated functions exposed here for usage in other files, documented in parser3.c */
void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols);
Params * load_json(const char * file_name);
void free_params(Params * params);


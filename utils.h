
#pragma once
#include "set.h"

/* Error MACRO */
#define DEBUG_ERROR(msg) fprintf(stderr, "Error: %s\nIn file: %s, line: %d, function: %s\n", msg, __FILE__, __LINE__, __func__)

int calculate_steps(set_t * params, char * min_key, char * max_key, char * r_key);


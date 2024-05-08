
#pragma once

#include "utils.h"
#include "commonmacros.h"

/* Keep all parameters extracted from .json file in clean, compact struct, which will be passed to A* */
typedef struct {

    float r;
    float s;
    float x_max;
    float x_min;
    float y_min;
    float y_max;
    float x_g;
    float y_g;
    float x_s;
    float y_s;
    float g_max;

    int s_div_r;
    int xs_steps;
    int ys_steps;
    int xr_steps;
    int yr_steps;
    int num_obstacles;
    int num_obstacle_parameters;

    float ** elliptical_obstacles;
    float ** g_image;

} Params;

/* Main encapsulated functions exposed here for usage in other files */
void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols);
Params * load_json(const char * file_name);



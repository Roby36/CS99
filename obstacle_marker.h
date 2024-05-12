
#pragma once 

#include <stdio.h>
#include <complex.h>
#include <math.h>
#include "parser.h"

// Define the type for complex numbers
typedef double complex Complex;

/* Opaque type for obstacle marker function */
typedef struct F F_t;

F_t * extract_obstacle_marker_func_params(Params * params);
void delete_obstacle_marker_func_params(F_t * F);
Complex L(Complex z1, Complex z2, F_t * F);
Complex calculate_path_integral(float ** test_path, int num_points, F_t * F);




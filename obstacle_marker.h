
#pragma once 

#include <stdio.h>
#include <complex.h>
#include <math.h>

#ifdef HC_UT // parser is only necessary for the unit test program
#include "parser.h"
#endif

#define NBR_OFF_COUNT 3 // neighbors per dimension

#include "commonmacros.h"
#include "utils.h"

typedef double complex Complex;

/* Opaque type for obstacle marker function */
typedef struct F F_t;

/* Contructor & destructor */
F_t * initialize_obstacle_marker_func_params(Params * params, float float_tol);
void delete_obstacle_marker_func_params(F_t * F);

/*** Main function to get an arbitrary Lvalue corresponding to some arbitarry edge ((x_c, y_c), (x_n, y_n)) ***/
Complex get_Lval(F_t * F, int x_c, int y_c, int x_n, int y_n); 

/* function to calculate integral along some arbitrary path */
Complex calculate_path_integral(float ** test_path, int num_points, F_t * F);

/* Simple getters */
Complex * get_residues(F_t * F);
Complex * get_obstacle_markers(F_t * F);


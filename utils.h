
#pragma once

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <complex.h>
#include <ctype.h>  // For isspace()

typedef double complex Complex;

/******** Macros ************/
#define SQRT2 1.4142135623730951
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define DEBUG_ERROR(msg) fprintf(stderr, "Error: %s\nIn file: %s, line: %d, function: %s\n", msg, __FILE__, __LINE__, __func__)

#define D_ADJACENT(x1, y1, x2, y2) \
    (((x1) == (x2) && (y1) == (y2)) ? 0.0f : \
     ((x1) != (x2) && (y1) != (y2)) ? SQRT2 : \
     1.0f)


#define CALCULATE_ELAPSED_TIME(start, end, elapsed) do { \
    gettimeofday(&end, NULL); \
    seconds = (end.tv_sec - start.tv_sec); \
    microseconds = (end.tv_usec - start.tv_usec); \
    elapsed = seconds + microseconds * 1e-6; \
} while(0)

/*********** Params *************
 * Main struct holding all parameters extracted from .json file,
 * which will be passed around all the modules to share the state of the environment 
*/
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

    float ** test_path_1;
    float ** test_path_2;
    int steps_path1;
    int steps_path2;

} Params;

/********* inline utility functions **********/

static inline int calculate_steps(float min, float max, float step_size) {

    // apply the equation to calculate the steps
    int steps = (int)((max - min) / step_size);
    return steps;
}

static inline bool cmplx_compare(Complex z1, Complex z2, double abs_tol) {

    return (
        fabs(creal(z2) - creal(z1)) < abs_tol &&
        fabs(cimag(z2) - cimag(z1)) < abs_tol
    );
}

static inline bool is_accessible(Params * params, int x, int y, float float_tol) {

    return (
        x >= 0 && y >= 0 &&
        x < params->xs_steps && y < params->ys_steps &&
        params->g_image != NULL &&
        params->g_image[(params->s_div_r) * y][(params->s_div_r) * x] >= float_tol
    );

}



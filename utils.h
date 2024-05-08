
#pragma once

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <ctype.h>  // For isspace()

/* Macros */
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


struct Params;

static inline int calculate_steps(float min, float max, float step_size) {

    // apply the equation to calculate the steps
    int steps = (int)((max - min) / step_size);
    return steps;
}


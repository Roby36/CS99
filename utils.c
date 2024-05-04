
#include "utils.h"
#include "set.h"

int calculate_steps(set_t * params, char * min_key, char * max_key, char * r_key) {

    // first retrieve the basic float values required from the set
    float min = * (float *) set_find(params, min_key);
    float max = * (float *) set_find(params, max_key);
    float r   = * (float *) set_find(params, r_key);

    // apply the equation to calculate the steps
    int steps = (int)((max - min) / r);
    return steps;
}
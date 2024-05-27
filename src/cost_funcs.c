
#include "cost_funcs.h"

/****************** cost / heuristic function implementations ****************+*/

float len(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    return (params->s * D_ADJACENT(x_1, y_1, x_2, y_2)); // Use s as scaling factor
}

float g_image_riemann_sum(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    // Determine what dimensions changed
    int dx = (x_1 != x_2);
    int dy = (y_1 != y_2);
    // Scale up the discrete s-relative coordinates to the corresponding r-relative indices of g_image
    int x_0 = (params->s_div_r) * x_1;
    int y_0 = (params->s_div_r) * y_1;

    // Now compute the sum of the function at all the (s/r) steps
    float rs = 0.0f;
    for (int i = 0; i < params->s_div_r; i++) {
        /** ensure that we are not accessing g_image out of bounds */
        int y = y_0 + (i * dy);
        int x = x_0 + (i * dx);
        if (x < 0 || x >= params->xr_steps || y < 0 || y >= params->yr_steps) {
            #ifdef ASTDBG
            DEBUG_ERROR("Attempt to access g_image matrix out of bounds");
            #endif
            continue; // skip iteration
        }
        rs += params->g_image[y][x];
    }
    // Multiply by the average lengh of each r-step to get the uncertainty accumulated 
    float u = params->r * D_ADJACENT(x_1, y_1, x_2, y_2) * rs;

    return u;
}

float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config) 
{
    float u = g_image_riemann_sum(x_1, y_1, x_2, y_2, params); // uncertainty accumulation
    float l = len(x_1, y_1, x_2, y_2, params);  // path length

    // Finally take a linear combination using the fixed parameters a,b
    float c = (config->a * u) + (config->b * l);
    return c;
}

float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config) 
{
    // First determine the total steps in the x and y directions
    int dx = abs(x_g - x_n);
    int dy = abs(y_g - y_n);
    // Then we take as many steps possible diagonally, and the rest in the remaining straight line
    float l = params->s * ((SQRT2 - 1) * MIN(dx,dy) + MAX(dx, dy)); // each step is also of size s
    // We multiply by the parameter b to ensure a tight heuristic compared to the cost function defined above
    float h = config->b * l;
    return h;
}

float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config)
{
    return 0.0f;
}



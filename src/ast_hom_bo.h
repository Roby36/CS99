
#include "commonmacros.h"
#include "astar_homotopies.h"
#include "parser.h"

#define MAX_HC_PER_TABLE 3  // Maximum number of homotopy classes per table, determined by latex rendering issues 

/*************** boundary_optimization *********************
 * 
 * This function sets one of the two linear coefficients a,b of the cost function,
 * that we will refer to as the fixed configuration parameter, to 1.0, 
 * whilst gradually increasing the other coefficient, which we will refer to as the variable configuration parameter.
 * Because the relative proportion between these two parameters is what ultimately determines
 * the behavior of the robot under A*, this function essentially turns the problem from a two-variable 
 * problem into a one-variable problem. 
 * 
 * When the variable configuration parameter is set to the minimum of zero, 
 * we record the benchmark value for what we call the fixed path measure (for instance, this would 
 * be path length when the variable configuration parameter is a, and uncertainty when the variable 
 * configuration parameter is b). As the variable configuration parameter gradually increases, 
 * the fixed path measure is expected to steadily increase, as it gets traded off for a 
 * decrease in the variable path measure. The algorithm is given a maximum acceptable fixed path measure, 
 * relative to the benchmark, within which a path is considered admissible (for example, 
 * 30% beyond path length), and terminates as soon as the current fixed path measure surpasses that value.
 * 
 * This extended version of the algorithm applies the reasoning above to each homotopy class
 * concurrently. 
 * 
 * Inputs:
 *  - Standard inputs for A_star_homotopies (note that we will require the zero heuristic here)
 *  - pointers to variable configuration and fixed configuration parameters
 *  - pointer to methods extracting the pointer to the variable path measure and fixed path measure 
 *    within an arbitrary path struct. (This is essentially a way to generalize for the desired path property, 
 *    thereby making the code reusable. We could also use struct offset values, but this method seems more robust to later changes).
 *  - The maximum admissible percentage for the fixed path metric, beyond the benchmark value 
 *  - The starting value for the variable configuration parameter
 *  - The multiplier for the variable configuration parameter between iterations, or runs of A*
 *  - A scale factor, changing the magnitudes of the fixed and variable parameters whilst maintaining their relative ratios
 *  - The maximum number of iterations, or runs of A* 
 *  - Title for the output LaTex table, which gets directly printed to stdout
 * 
 * Outputs:
 *  - bt_paths: array of optimal paths for each homotopy class, for each iteration
 *  - fix_config_vals, var_config_vals: arrays containing the values for these parameters at each iteration
 *  - curr_it: set to the value of the final iteration
 *
 * NOTE: Caller responsible for allocating BO_Params struct with inputs and free'ing it after the call
 *       by calling bo_clean_up
 *      
 */
struct BO_Params {
// constant input parameters 
    float * var_config;  
    float * fix_config; 
    float * (* var_path_met)(Backtrack_Path *);   
    float * (* fix_path_met)(Backtrack_Path *);   
    float bound;      
    float var_start;  
    float var_mult;
    float scale_fact;   
    int MAX_IT;
    char * table_title; 
// output parameters 
    Backtrack_Path *** bt_paths;
    float * fix_config_vals;
    float * var_config_vals;
    int curr_it;    // gets set to total iterations performed by the algo
};

void boundary_optimization(
    struct A_star_homotopies_args * astar_args,
    struct BO_Params * bo
);


/* Getters / setters for fixed and variable path metrics */
float * g_image_getter(Backtrack_Path * bt) {
    float * g_image_ptr = &(bt->g_image_riemann_tot);
    return g_image_ptr;
}

float * path_length_getter(Backtrack_Path * bt) {
    float * length_ptr = &(bt->absolute_length);
    return length_ptr;
}

// Prints function directly in LaTex format
void print_LaTex_table(
    struct A_star_homotopies_args * astar_args,
    struct BO_Params * bo,
    char * table_title
);

// Function to clean up after running boundary_optimization 
void bo_clean_up(
    struct A_star_homotopies_args * astar_args, 
    struct BO_Params * bo
);



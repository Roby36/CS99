
#include "commonmacros.h"
#include "astar.h"
#include "parser.h"

/* This module runs several times A* with different a,b configurations to optimize either for uncertainty or path length */

void boundary_optimization(
/* A* parameters */
    Params * params,
    Config * config,
    Optimal_Path * opt_path,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ),
    float float_tol,
/* boundary-optimization-specific parameters */
    float * var_config,     // variable configuration parameter to optimize (e.g. a for uncertainty)
    float * fix_config,     // fixed configuration parameter to control (e.g. b for path length)
    float * var_path_met,   // variable path metric to optimize (e.g. uncertainty relating to a)
    float * fix_path_met,   // fixed path metric to control (e.g. path length relating to b)
    const float bound,      // The maximum tolerance allowed beyond the optimized fixed path metric
    const float var_start,  // starting value for variable configuration parameter
    const float var_mult,   // multiplicator for variable configuration parameter
    const int MAX_IT        // maximum iterations (and thus A* runs permitted)
) {
    float var_prev, var_curr;   // previous and current values of variable path metric (e.g. uncertainty)
    float fix_0, fix_curr;      // benchmark for the fixed path metric (e.g. length), and the varying current value
    int curr_it = 0;            // current loop iteration

    /* First we set the fixed path metric benchmark by running A* with a pure cost function */
    /** IMPORTANT: We faithfully assume that A* does not modify the config struct */
    *(fix_config) = 1.0f;   // e.g. b = 1, a = 0 to initialize fixed and variable configuration parameters
    *(var_config) = 0.0f;
    A_star(params, config, opt_path, h, c, float_tol); /** IMPORTANT: A* allocates internal point array of P, which must be free'd to avoid problematic leaks */
    
    fix_curr = fix_0 = *(fix_path_met); // record benchmark fixed path metric (e.g. length), and initialize current 
    var_prev = INFINITY;    // this guarantees that var_curr < var_prev at first iteration
    var_curr = *(var_path_met); // initialize the current variable path metric (e.g. uncertainty) to suboptimal value 

    /* Now we iteratively increase variable configuration parameter's weight, until we remain within the desired bounds for the fixed path metric */
    *(var_config) = var_start;
    while (fix_curr < (1.0f + bound) * fix_0) {

        /* Check that we haven't exceeded the maxiumum allowed iterations*/
        if (curr_it == MAX_IT) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as max_it = %d was reached", MAX_IT);
            #endif
            break;
        }

        free_point_array(opt_path);    // clean up the internal list of the path since it gets allocated at each A* call
        A_star(params, config, opt_path, h, c, float_tol);    // re-allocate and update path with new variable configuration parameter
        fix_curr = *(fix_path_met); // update current fixed path metric (e.g. length)
        var_curr = *(var_path_met); // update current variable path metric (e.g. uncertainty)

        /* Now, if we have had no variable path metric improvement (decrease) despite increasing the corresponding configuration parameter, we declare convergence */
        /* Because of the nature of floating points, this should only be possible if we had the same path */
        if (var_curr == var_prev) {
            #ifdef BO_LOG
            printf("\nBoundary optimization converged as variable path metric showed no improvement at iteration %d"
                   " with variable configuration parameter %f\n", curr_it, *(var_config));
            #endif
            break;
        }
        /* If we get increase in variable path metric, we print an unexpected output error message */
        else if (var_curr > var_prev) {
            #ifdef BO_LOG
            printf("\nBoundary optimization: unexpected output as variable path metric increased at iteration %d\n", curr_it);
            #endif
        }

        var_prev = var_curr;        // keep track of latest uncertainty update
        curr_it++;
        *(var_config) *= var_mult;  // scale up the a-parameter by the multiple
    }

    /* By the time we reach this point we have an allocated path, and thus can print the output */
    #ifdef BO_LOG
    printf("\n boundary_optimization terminated with inputs:\n"
           "\tbound = %f; var_start = %f; var_mult = %f\n"
           "\nMinimized variable path metric value: %f\n"
           "\twith fixed path metric value of %f, and fixed path metric benchmark %f\n"
           "\twith cost function linear parameters fcp = %f, vcp = %f ",
        bound, var_start, var_mult, var_curr, fix_curr, fix_0, *(fix_config), *(var_config)
    );
    #endif 
}



#ifdef BO_UT

int main() {

    // Hard-coded constants
    const char * input_json_path = "./params.json";
    const float float_tol = 0.05;
    const float bound = 0.3;
    const float var_start = 0.01;
    const float var_mult = 4.0f;
    const int MAX_IT = 100;

    // Local variables for A*
    Params * params;
    Optimal_Path * opt_path;
    Config * config;

    params = load_json(input_json_path);
    if (params == NULL) {
        DEBUG_ERROR("load_json / extract_parameters returned NULL, exiting...");
        exit(1);
    }

    // Allocate memory for the path struct & configuration
    opt_path = malloc(sizeof(Optimal_Path));
    config   = malloc(sizeof(Config));

    boundary_optimization(
/* A* parameters */
    params, config, opt_path, heuristic, edge_cost, float_tol,
/* boundary-optimization-specific parameters */
    &config->a,     // variable configuration parameter to optimize (e.g. a for uncertainty)
    &config->b,     // fixed configuration parameter to control (e.g. b for path length)
    &opt_path->g_image_riemann_tot,   // variable path metric to optimize (e.g. uncertainty relating to a)
    &opt_path->len,         // fixed path metric to control (e.g. path length relating to b)
    bound,      // The maximum tolerance allowed beyond the optimized fixed path metric
    var_start,  // starting value for variable configuration parameter
    var_mult,   // multiplicator for variable configuration parameter
    MAX_IT      // maximum iterations (and thus A* runs permitted)
);

    // fully allocated opt_path and config structs now hold the optimized values

    // Clean up
    free(params);
    free(config);
    free_point_array(opt_path);
    free(opt_path);

    return 0;
}

#endif












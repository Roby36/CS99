
#include "commonmacros.h"
#include "astar_homotopies.h"
#include "parser.h"

/* This module runs several times A* with different a,b configurations to optimize either for uncertainty or path length */

/* Getters / setters for fixed and variable path metrics */
float * g_image_getter(Backtrack_Path * bt) {
    float * g_image_ptr = &(bt->g_image_riemann_tot);
    return g_image_ptr;
}

float * path_length_getter(Backtrack_Path * bt) {
    float * length_ptr = &(bt->absolute_length);
    return length_ptr;
}

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
 * 
 * Outputs:
 *  - Array var_curr, keeping the final values of the variable path metric for each homotopy class
 *  - Array fix_curr, keeping the final values of the fixed path metric for each homotopy class
 *  - Array fix_0, keeping the fixed path metric benchmarks for each homotopy class
 *  - Relevant configuration values of a,b are currently logged to stdout, but can be saved or returned 
 *    for future use
 */
void boundary_optimization(
/* A* parameters */
    struct A_star_homotopies_args * astar_args,
/* boundary-optimization-specific parameters */
    float * var_config,     
    float * fix_config,     
    float * (* var_path_met)(Backtrack_Path *),   
    float * (* fix_path_met)(Backtrack_Path *),   
    const float bound,      
    const float var_start,  
    const float var_mult,
    const float scale_fact,   
    const int MAX_IT        
) {
    // previous and current values of variable path metric (e.g. uncertainty), for each homotopy class
    Backtrack_Path * var_prev [astar_args->max_hom_classes]; 
    Backtrack_Path * var_curr [astar_args->max_hom_classes];   

    // benchmark for the fixed path metric (e.g. length), and the varying current value, for each homotopy class
    Backtrack_Path * fix_0    [astar_args->max_hom_classes];
    Backtrack_Path * fix_curr [astar_args->max_hom_classes];  

    // current main loop iteration
    int curr_it = 0;            

    /* First we set the fixed path metric benchmark by running A* with a pure cost function */
    /** IMPORTANT: We faithfully assume that A* does not modify the config struct */
    *(fix_config) = (1.0f / scale_fact);   // e.g. b = 1, a = 0 to initialize fixed and variable configuration parameters
    *(var_config) = 0.0f;

    // Call A* with intial config to set the entries in target_hom_classes_ptr to the benchmarks for each homotopy class
    A_star_homotopies(astar_args); 

    // Now allocate all the pointers to paths within the arrays, holding tuple values corresponding to each sequential homotopy class in the benchmark list
    hom_class_t * bench_it = (*(astar_args->target_hom_classes_ptr))->head;
    int curr_index = 0;
    while (bench_it != NULL) {

        // Allocate path pointers for the current entries of the arrays
        Backtrack_Path * var_prev_entry = var_prev [curr_index] = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));
        Backtrack_Path * var_curr_entry = var_curr [curr_index] = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));
        Backtrack_Path * fix_0_entry    = fix_0    [curr_index] = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));
        Backtrack_Path * fix_curr_entry = fix_curr [curr_index] = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));

        /* Now set attributes for each entry as desired, using the getters / setters to retrieve and modify the desired path attributes */

        // Initialize the fixed path metric of both entries for current fixed value and fix_0 to the current benchmark ones
        * (fix_path_met(fix_curr_entry)) = * (fix_path_met(fix_0_entry)) = * (fix_path_met(bench_it->backtrack));
        // Initialize the previous value of the variable path metric at INFINTY, guaranteeing that the variable path metric decreases at first iteration
        * (var_path_met(var_prev_entry)) = INFINITY;
        // Initialize the current value of the variable path metric to the suboptimal value returned by the benchmark iteration
        * (var_path_met(var_curr_entry)) = * (var_path_met(bench_it->backtrack));

        // Slide forwards through the list of homotopy classes
        curr_index++;
        bench_it = bench_it->next;
    }

    /* Now we enter the main loop where we iteratively increase the variable configuration parameter's weight metric */
    *(var_config) = (var_start / scale_fact);
    bool homotopy_classes_updated = true; // flag signaling whether any homotopic classes have been updated, thus whether further iterations are still needed
    
    while (true) {

        /** NOTE: A_star_homotopies keeps intact the relative location of each homotopy class within the benchmark_hom_classes list,
         * and thus updates its corresponding values in place. Hence we can expect each homotopy class to be located constantly at the same array index. 
         * While the Back_Track oject is destroyed at each iteration, A_star_homotopies writes to params.json each path into the goal vertex it discovers.
         * For more information on the implications of this call see A_star_goal_check in astar.c 
        */        
        A_star_homotopies(astar_args);
        
        // Update each homotopy class within the arrays with the update path metrics from A*
        hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
        curr_index = 0;
        homotopy_classes_updated = false; 

        while (hom_class_it != NULL) {

            // Extract current entries for the current homotopy class
            Backtrack_Path * var_prev_entry = var_prev [curr_index];
            Backtrack_Path * var_curr_entry = var_curr [curr_index];
            Backtrack_Path * fix_0_entry    = fix_0    [curr_index];
            Backtrack_Path * fix_curr_entry = fix_curr [curr_index];

            /* First update current fixed an current variable path metrics with most recent updated values from A* */
            * (fix_path_met(fix_curr_entry)) = * (fix_path_met(hom_class_it->backtrack)); 
            * (var_path_met(var_curr_entry)) = * (var_path_met(hom_class_it->backtrack)); 
        
            /* Check that this homotopy class's fixed path metric is within bounds before proceeding */ 
            if (* (fix_path_met(fix_curr_entry)) > (1.0f + bound) * (* (fix_path_met(fix_0_entry))) ) {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization: " 
                    "skipping homotopy class (%.2f + %.2fi) / index %d"
                    " at iteration %d becuase the fixed path metric exceeded bounds, "
                    " with variable configuration parameter %.2f\n",
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_index,
                    curr_it, *(var_config)
                );
                #endif

                goto hom_class_inner_loop_end;  // continue to next iteration
            }
            /* Checking for path convergence, which occurs when there is no change in variable path metric */
            /** NOTE: currently skipping convergence check for diagnostic purposes
            if ( * (var_path_met(var_curr_entry)) == * (var_path_met(var_prev_entry))) {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization converged " 
                    "for homotopy class (%.2f + %.2fi) / index %d"
                    " as variable path metric showed no improvement at iteration %d "
                    " with variable configuration parameter %.2f\n",
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_index,
                    curr_it, *(var_config));
                #endif

                goto hom_class_inner_loop_end;  // continue to next iteration
            }
            */
            /* If we get an increase in variable path metric, we print an unexpected output error message */
            else if ( * (var_path_met(var_curr_entry)) > * (var_path_met(var_prev_entry))) {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization: " 
                    "unexpected output for homotopy class (%.2f + %.2fi) / index %d" 
                    " as variable path metric increased at iteration %d\n", 
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_index, curr_it);
                #endif
            }

            /* End-of-loop routine */

            homotopy_classes_updated = true;    // if we make it to here, flag an update in at least one homotopy class
            * (var_path_met(var_prev_entry)) = * (var_path_met(var_curr_entry)); // update previous variable path measure

            // Slide forwards
            hom_class_inner_loop_end:
            curr_index++;
            hom_class_it = hom_class_it->next;
        }

        /* End-of-loop routine: check two loop-break conditions */
        if (!homotopy_classes_updated) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as no homotopy clases have been updated at iteration %d\n", curr_it);
            #endif
            break;
        }
        if (++curr_it >= MAX_IT) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as max_it = %d was reached\n", MAX_IT);
            #endif
            break;
        }

        // Guarantees that A* is performed for the updated value 
        *(var_config) *= var_mult;  
    }

    /* Here the main routine terminated, and we optionally log the output */

    #ifdef BO_LOG 
    printf(
        "\nboundary_optimization terminated with inputs:\n"
        "\tbound = %.2f; var_start = %.2f; var_mult = %.2f\n",
        bound, var_start, var_mult
    );
    #endif

    // Now iterate throughout all the homotopy classes and indices
    hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
    curr_index = 0;
    while (hom_class_it != NULL) {

        #ifdef BO_LOG 
        printf(
            "\nBoundary optimization for homotopy class (%.2f + %.2fi) / index %d:\n"
            "\tFinal minimized variable path metric value: %.2f\n"
            "\twith final fixed path metric value of %.2f, and fixed path metric benchmark of %.2f\n"
            "\twith cost function linear parameters fcp = %.2f, vcp = %.2f \n",
        creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_index,
        * (var_path_met(var_curr [curr_index])), 
        * (fix_path_met(fix_curr [curr_index])),
        * (fix_path_met(fix_0    [curr_index])),
        *(fix_config), *(var_config)
        );
        #endif 

        /*** NOTE: Here we clean-up the arrays, but if needed in the future we could potentially return some of these arrays. */
        free(var_prev [curr_index]); 
        free(var_curr [curr_index]); 
        free(fix_0    [curr_index]); 
        free(fix_curr [curr_index]);

        // Slide forward
        curr_index++;
        hom_class_it = hom_class_it->next;
    }   
}


/************ Unit testing ******************
 * 
 * Below is illustrated an example use case of the function,
 * showing all the allocations, initializations, and clean up
 * routines necessary
*/

#ifdef BO_UT

int main(int argc, char *argv[]) {

    /* Hard-coded constants */
    const float float_tol = 0.05;
    const double abs_tol = 1.0;
    const float var_start = 0.01;
    const float scale_fact = 1.0f;
  
    /* Inputs initialized from the command line */
    char * input_json_path;
    float bound = 0.0f;
    float var_mult = 0.0f;
    int MAX_IT = 0;
    int max_expandible_states_mult = 0;

    /* Local variables for A* */
    Params * params;
    Config * config;

    // Parsing command-line arguments
    if (argc != 6) { 
        fprintf(stderr, "Usage: %s <input_json_path> <bound> <var_mult> <MAX_IT> <max_expandible_states_mult> \n", argv[0]);
        return 1;  
    }

    input_json_path = (char *) malloc(strlen(argv[1]) + 1);
    if (input_json_path == NULL) {
        fprintf(stderr, "Memory for <input_json_path> string allocation failed\n");
        return 1;
    }
    strcpy(input_json_path, argv[1]);

    PARSE_FLOAT(argv[2], bound, 2);
    PARSE_FLOAT(argv[3], var_mult, 3);
    PARSE_INT(argv[4], MAX_IT, 4);
    PARSE_INT(argv[5], max_expandible_states_mult, 5);

    params = load_json(input_json_path);
    if (params == NULL) {
        DEBUG_ERROR("load_json / extract_parameters returned NULL, exiting...");
        exit(1);
    }

    config = malloc(sizeof(Config));
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);
    const int max_hom_classes = params->num_obstacles + (params->num_obstacles / 2); // 50% more than the number of obstacles
    hom_classes_list_t * target_hom_classes = NULL; 

    // Log the parsed parameters
    printf(
        "Running boundary_optimization with inputs from command-line " 
        "bound = %f, var_mult = %f, MAX_IT = %d, max_expandible_states_mult = %d\n", 
        bound, var_mult, MAX_IT, max_expandible_states_mult
    ); 

    // Wrap up all the inputs into the struct

    struct A_star_homotopies_args  * astar_args = malloc(sizeof(struct A_star_homotopies_args));

    astar_args->params = params;
    astar_args->config = config;
    astar_args->h = zero_heuristic;
    astar_args->c = edge_cost;
    astar_args->float_tol = float_tol;
    astar_args->abs_tol = abs_tol;
    astar_args->F = F;
    astar_args->target_hom_classes_ptr = &target_hom_classes;
    astar_args->max_hom_classes = max_hom_classes;
    astar_args->max_expandible_states_mult = max_expandible_states_mult;

/* Input initialization complete: now invoking the main function */
    boundary_optimization(
    /* A* parameters */
    astar_args,
    /* boundary-optimization-specific parameters */
    &config->a,     // variable configuration parameter to optimize (e.g. a for uncertainty)
    &config->b,     // fixed configuration parameter to control (e.g. b for path length)
    g_image_getter,
    path_length_getter,
    bound,      // The maximum tolerance allowed beyond the optimized fixed path metric
    var_start,  // starting value for variable configuration parameter
    var_mult,   // multiplicator for variable configuration parameter
    scale_fact,
    MAX_IT        // maximum iterations (and thus A* runs permitted)
    );

    // Clean up
    free(params);
    free(config);
    delete_obstacle_marker_func_params(F);
    free_hom_classes_list(target_hom_classes);
    free(input_json_path);
    free(astar_args);

    return 0;
}

#endif




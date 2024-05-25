
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
    // <metric_type> (iteration # -> (homotopy class -> Backtrack path *))
    Backtrack_Path *** fix_met_vals = (Backtrack_Path ***) malloc((MAX_IT + 1) * sizeof(Backtrack_Path **));
    Backtrack_Path *** var_met_vals = (Backtrack_Path ***) malloc((MAX_IT + 1) * sizeof(Backtrack_Path **));
    // <config_type> (iteration # -> config value)
    float * fix_config_vals = (float *) malloc((MAX_IT + 1) * sizeof(float));
    float * var_config_vals = (float *) malloc((MAX_IT + 1) * sizeof(float));
 
    // Initialize configuration parameters for iteration, and write them to the arrays 
    int curr_it = 0; /** IMPORTANT: We will faithfully assume that A* does not modify the config struct */    
    bool homotopy_classes_updated; // flag signaling whether any homotopic classes have been updated, thus whether further iterations are still needed
    *(fix_config) = (1.0f / scale_fact);  // e.g. b = 1, a = 0 to initialize fixed and variable configuration parameters
    *(var_config) = 0.0f;
    int curr_hom_index;
    
    while (true) {

        if (curr_it == 1) {
            *(var_config) = (var_start / scale_fact);
        }

        // Update cnfiguration values for current iteration
        fix_config_vals[curr_it] = *(fix_config);
        var_config_vals[curr_it] = *(var_config);

        // Allocate arrays of path pointers for each homotopy class of this iteration
        fix_met_vals[curr_it] = (Backtrack_Path **) malloc((astar_args->max_hom_classes) * sizeof(Backtrack_Path *));
        var_met_vals[curr_it] = (Backtrack_Path **) malloc((astar_args->max_hom_classes) * sizeof(Backtrack_Path *));

        /** NOTE: A_star_homotopies keeps intact the relative location of each homotopy class within the benchmark_hom_classes list,
         * and thus updates its corresponding values in place. Hence we can expect each homotopy class to be located constantly at the same array index. 
         * While the Back_Track oject is destroyed at each iteration, A_star_homotopies writes to params.json each path into the goal vertex it discovers.
         * For more information on the implications of this call see A_star_goal_check in astar.c 
        */        
        A_star_homotopies(astar_args);
        
        // Update each homotopy class within the arrays with the update path metrics from A*
        hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
        curr_hom_index = 0;
        homotopy_classes_updated = false; 

        while (hom_class_it != NULL) {

            // Get currently unallocated entries for current iteration and homotopy class
            Backtrack_Path ** fix_curr_entry_ptr = &(fix_met_vals[curr_it][curr_hom_index]);
            Backtrack_Path ** var_curr_entry_ptr = &(var_met_vals[curr_it][curr_hom_index]);

            // Check that this homotopy class's fixed path metric is within benchmark bounds
            if ((curr_it != 0) && // ensure that we skip the condition for the benchmark itself 
                *(fix_path_met(hom_class_it->backtrack)) > (1.0f + bound) * (* (fix_path_met(fix_met_vals[0][curr_hom_index]))) ) 
            {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization: " 
                    " skipping homotopy class (%.2f + %.2fi) / index %d, setting Backtrack_Path = NULL, "
                    " at iteration %d becuase the fixed path metric exceeded bounds, "
                    " with variable configuration parameter %.2f\n",
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_hom_index,
                    curr_it, *(var_config)
                );
                #endif

                // Set entries to NULL to show that the boundary condition was not satisfied, and skip over the allocations 
                *fix_curr_entry_ptr = *var_curr_entry_ptr = NULL;
                goto hom_class_inner_loop_end;  // continue to next iteration
            }

            /** OPTIONAL: Check for path convergence or unexpected rise by comparing with previous values of variable path metric*/

            // If the bound condition was passed, then allocate paths, and set them to their current values 
            *fix_curr_entry_ptr = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));
            *var_curr_entry_ptr = (Backtrack_Path *) malloc(sizeof(Backtrack_Path));
            *(fix_path_met(*fix_curr_entry_ptr)) = *(fix_path_met(hom_class_it->backtrack));  // benchmark for curr_hom_index = fix_met_vals[0][curr_hom_index] 
            *(var_path_met(*var_curr_entry_ptr)) = *(var_path_met(hom_class_it->backtrack));
            homotopy_classes_updated = true;    // if we make it to here, flag an update in at least one homotopy class

            // End of loop: iterating to next homotopy class
            hom_class_inner_loop_end:
            curr_hom_index++;
            hom_class_it = hom_class_it->next;
        }

        /* End-of-loop routine: check two loop-break conditions */
        if (++curr_it > MAX_IT) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as max_it = %d was reached\n", MAX_IT);
            #endif
            break;
        }

        if (!homotopy_classes_updated) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as no homotopy clases have been updated at iteration %d\n", curr_it);
            #endif
            break;
        }
        
        // Guarantees that A* is performed for the updated value 
        *(var_config) *= var_mult;  
    }


    /* Main routine terminated: Logging and cleaning up simultaneously */
    #ifdef BO_LOG 
    printf(
        "\nboundary_optimization terminated with inputs:\n"
        "\tbound = %.2f; var_start = %.2f; var_mult = %.2f\n",
        bound, var_start, var_mult
    );
    #endif


    /*
    
    printf("\\begin{table}[h!]\n");
    printf("\\centering\n");
    printf("\\begin{tabular}{|c|c|");
    for (int j = 0; j < astar_args->max_hom_classes; j++) {
        printf("c|c|"); // Two columns per homotopy class
    }
    printf("}\n\\hline\n");
    printf("Iteration & Fixed Config & Variable Config ");

    for (int j = 0; j < astar_args->max_hom_classes; j++) {
        printf("& Fixed Path Metric %d & Variable Path Metric %d ", j, j);
    }
    printf("\\\\ \\hline\n");

    // Iterating over each iteration and each homotopy class
    for (int it = 0; it < curr_it; it++) {

        printf("%d & %.2f & %.2f", it, fix_config_vals[it], var_config_vals[it]);  // assuming pointers to floats for configs

        for (int curr_hom_index = 0; curr_hom_index < astar_args->max_hom_classes; curr_hom_index++) {

            // NULL-checks
            

            float fixed_metric    = *(fix_path_met(fix_met_vals[it][curr_hom_index]));
            float variable_metric = *(var_path_met(var_met_vals[it][curr_hom_index]));
            printf(" & %.2f & %.2f", fixed_metric, variable_metric);
        }
        printf("\\\\ \\hline\n");
    }
    printf("\\end{tabular}\n");
    printf("\\caption{Summary of configurations and path metrics across iterations}\n");
    printf("\\label{tab:metrics_summary}\n");
    printf("\\end{table}\n");

    */


    /* Clean-up procedure */
    for (int it = 0; it < curr_it; it++) {
        hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
        curr_hom_index = 0;
        while (hom_class_it != NULL) {
            // Check that this Backtrack path was actually allocated 
            if (fix_met_vals[it][curr_hom_index] != NULL) {
                free(fix_met_vals[it][curr_hom_index]);
            }
            if (var_met_vals[it][curr_hom_index] != NULL) {
                free(var_met_vals[it][curr_hom_index]);
            }

            // Slide forward
            curr_hom_index++;
            hom_class_it = hom_class_it->next;
        }

        free(fix_met_vals[it]);
        free(var_met_vals[it]);
    }

    free(fix_met_vals);
    free(var_met_vals);
    free(fix_config_vals); 
    free(var_config_vals);

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
    #ifdef AST_HC_DBG
    printf(
        "Running boundary_optimization with inputs from command-line " 
        "bound = %f, var_mult = %f, MAX_IT = %d, max_expandible_states_mult = %d\n", 
        bound, var_mult, MAX_IT, max_expandible_states_mult
    ); 
    #endif

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




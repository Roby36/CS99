
#include "astar_homotopies.h"


/****************** Main A* algorithm implementation **************************/

void A_star_homotopies(struct A_star_homotopies_args * args) 
{
    int expanded_states = 0;    // we consider a state expanded whenever it is dequeued from the closed set
    const int max_expandible_states = (args->max_expandible_states_mult) * (args->params->xs_steps) * (args->params->ys_steps);
    int filled_hom_classes = 0; 

    #ifdef ASTCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    #endif

    #ifdef AST_HC_DBG
    printf(
        "\nA* invoked with the inputs:\n"
        "\t(x_g, y_g) = (%f, %f)\n"
        "\tr = %f, s = %f\n"
        "\ta = %f, b = %f\n"
        "\tfloat tolerance = %f\n"
        "\tabsolute tolerance for homotopy classes: %f\n"
        "\tMaximum homotopy classes: %d\n"
        "\tMaximum expandible states: %d\n\n",
    args->params->x_g, args->params->y_g, args->params->r, args->params->s, 
    args->config->a, args->config->b, args->float_tol, args->abs_tol, args->max_hom_classes,
    max_expandible_states
    );
    #endif

    // Determine if this will be an initialization run
    const bool target_hom_classes_init = (*(args->target_hom_classes_ptr) == NULL);

    // Flag current target homotopy classes as not updated
    if (!target_hom_classes_init) {
        unflag_homotopy_classes(*(args->target_hom_classes_ptr));
    }

    if (args->params->g_image == NULL) {
        DEBUG_ERROR("params->g_image = NULL");
        return;
    }
    
    // Convert start coordinates to integre grid coordinates
    const int x_start_step = (int) (args->params->x_s - args->params->x_min) / args->params->s;
    const int x_goal_step  = (int) (args->params->x_g - args->params->x_min) / args->params->s;
    const int y_start_step = (int) (args->params->y_s - args->params->y_min) / args->params->s;
    const int y_goal_step  = (int) (args->params->y_g - args->params->y_min) / args->params->s;

    // Abort A* if we find start/goal to be inaccessible
    if (!is_accessible(args->params, x_start_step, y_start_step, args->float_tol) ||
        !is_accessible(args->params, x_goal_step,  y_goal_step,  args->float_tol)) {
        DEBUG_ERROR("start or goal state not accessible, A* aborted");
        return;
    }

    // Initialize vertices to default values (see hom_classes.c)
    hom_vertex_t *** S = allocate_grid(args);

    // initialize start homotopy class
    hom_vertex_t * start_vertex = S[y_start_step][x_start_step];
    hom_class_t * start_hom_class = hom_class_new(start_vertex, CMPLX(0.0, 0.0));
    start_hom_class->g_score = 0.0f;
    start_hom_class->f_score = args->h(
        x_start_step, y_start_step, x_goal_step, y_goal_step, args->params, args->config
    );
    hom_class_t * dummy_ptr;    // assume the start homotopy class trivially qualifies 
    insert_hom_class_sorted(start_vertex->hom_classes, start_hom_class, args->abs_tol, &dummy_ptr); 

    // initialize priority queue 
    open_set_t * open_set = open_set_new(
        hom_class_get_f_score,  // accessors for homotopy class
        hom_class_set_minheap_node
    ); /** CAREFUL: not checking for NULL pointer return here */
    insert_sorted(open_set, (void *) start_hom_class);

    // main loop for A*
    while (!open_set_is_empty(open_set)) {

        hom_class_t * curr_hom_class = dequeue_min(open_set);
        curr_hom_class->is_evaluated = true; // dequeue homotopy class & add it to the closed set
        
        // maximum state cap: ideally should not be required for termination
        if (++expanded_states > max_expandible_states) {
            #ifdef AST_HC_DBG
            printf("Maximum number of expandible states %d surpassed: breaking out of main loop...\n", max_expandible_states);
            #endif
            break;
        }

        // current x,y grid coordinates
        int x_c = curr_hom_class->endpoint_vertex->x_s;
        int y_c = curr_hom_class->endpoint_vertex->y_s;

        // Goal state subroutine
        if ((x_c == x_goal_step && y_c == y_goal_step))
        {      
            // expand, write to .json and log discovered path 
            expand_backtrack(args->params, curr_hom_class);
            write_path(curr_hom_class, args->config); // write path to ./params.json
            #ifdef AST_HC_DBG
            char * B_str = complex_list_to_string(curr_hom_class->endpoint_vertex->hom_classes);
            printf(
                "\nA* found new homotopic class to goal:\n"
                "\tL-value: (%.2f + %.2fi)\n"
                "\tcumulative length: %f\n" 
                "\tg_image Riemann sum (accumulated uncertainty): %f\n"
                "\nCurrent status of the graph search:\n"
                "\tTotal states expanded: %d\n"
                "\tHomotopy classes currently evaluated for the goal vertex: %s\n"
                "\tCurrent size of the open set (enqueued states): %d\n",
            creal(curr_hom_class->Lval), cimag(curr_hom_class->Lval), curr_hom_class->backtrack->absolute_length,
            curr_hom_class->backtrack->g_image_riemann_tot, expanded_states, B_str, open_set_size(open_set)
            );
            free(B_str);
            #endif

            // skip if there is no target class to update 
            if (target_hom_classes_init) {
                goto goal_check_end;
            }
            hom_class_t * target_hom_class = hom_class_get(*(args->target_hom_classes_ptr), curr_hom_class->Lval, args->abs_tol);
            if (target_hom_class == NULL) {
                goto goal_check_end;
            }

            // We update the just-extracted homotopy class using the following conventions
            //      curr_hom_class unaffected
            //      target set to current, except for the next pointer, required to preserve the structure of the target list
            //      deep copy of the backtrack path struct, so that only curr_hom_class keeps the pointer to the path

            hom_class_t * next = target_hom_class->next;    // preserve next pointer 
            free_backtrack(target_hom_class);               // clean up old backtrack          
            *(target_hom_class) = *(curr_hom_class);        // shallow copy
            target_hom_class->backtrack = NULL;             // re-expand path for target class
            expand_backtrack(args->params, target_hom_class);
            target_hom_class->next = next;                  // restore next pointer to preserve list structure       
            filled_hom_classes++;                           // mark filled class
    
        }   // End of goal state subroutine 
        goal_check_end:

        // Generating outneighbors
        for (int dx = -1; dx <= 1; dx++) {  // x offsets
        for (int dy = -1; dy <= 1; dy++) {  // y offsets
            // same state automatically skipped by the later is_evaluated condition
            int x_n = x_c + dx; // neighbouring state
            int y_n = y_c + dy;
            if (!is_accessible(args->params, x_n, y_n, args->float_tol)) {
                continue; 
            }

            hom_vertex_t * neighb_vertex = S[y_n][x_n]; // fetch some neighboring vertex 
            Complex neighb_Lval = curr_hom_class->Lval + get_Lval(args->F, x_c, y_c, x_n, y_n); // get its L-value
            
            // check if homotopy class corresponding to this descriptor was discovered already
            bool discovered_nbr_class = false;
            hom_class_t* nbr_hom_class = hom_class_get(neighb_vertex->hom_classes, neighb_Lval, args->abs_tol);
            // If not, initialize it to defaults (see hom_classes.c)
            if (nbr_hom_class == NULL) {
                discovered_nbr_class = true;
                nbr_hom_class = hom_class_new(neighb_vertex, neighb_Lval);
            }
            // These are preliminary checks intended only for already-discovered homotopy classes
            if (nbr_hom_class->is_evaluated) {
                continue;
            }            
            // If no better path found for this homotopy class, skip it
            float tent_g = curr_hom_class->g_score + args->c(   // g_score for reaching the neighbor through the current path
                x_c, y_c, x_n, y_n, args->params, args->config
            );
            if (tent_g >= nbr_hom_class->g_score) {
                continue;
            }

            // If we make it to here, then we must have one of the following two situations:
            // 1)  just-discovered nbr homotopy class that we want to update and try to add to the nbr vertex list if it qualifies
            // 2)  already-discovered hc extracted from the nbr vertex list that has passed the checks above, thus ready for update

            #ifdef ASTDBG
            printf("Updating some homotopy class at vertex (%d, %d) from g_score: %f to g_score: %f\n", neighb_vertex->x_s, neighb_vertex->y_s, neighb_vertex->g_score, tent_g);
            #endif
            nbr_hom_class->parent  = curr_hom_class;
            nbr_hom_class->g_score = tent_g;
            nbr_hom_class->f_score = tent_g + args->h(x_n, y_n, x_goal_step, y_goal_step, args->params, args->config);

            // handle new homotopy class
            if (discovered_nbr_class) {
                hom_class_t * truncated_element;
                bool inserted = insert_hom_class_sorted(
                    neighb_vertex->hom_classes, nbr_hom_class, args->abs_tol, &truncated_element
                );
                // discard nbr hom class if it didn't enter the list
                if (!inserted) {
                    free(nbr_hom_class);
                    continue;
                }
                // if nbr hom was inserted, we have to free the open set from the potential truncated element 
                if (truncated_element) {
                    remove_from_open_set(open_set, truncated_element->minheap_node);
                    free(truncated_element);
                }
            }
        
            // if homotopy class already in the min heap, dedrease key, else add it
            if (nbr_hom_class->minheap_node) {
                decrease_key(open_set, nbr_hom_class->minheap_node, nbr_hom_class->f_score);
            } else {
                insert_sorted(open_set, nbr_hom_class);
            }
        }
        }
    }   // End of main A* procedure


    // Handle initialization of target hom classes
    if (target_hom_classes_init) {    
        *(args->target_hom_classes_ptr) = S[y_goal_step][x_goal_step]->hom_classes; // write directly the discovered list
        filled_hom_classes =  args->max_hom_classes;    // if we made it to here we can assume this
    }

    // logging and cleaning up procedure
    #ifdef AST_HC_DBG
    char * hom_classes_str = complex_list_to_string(*(args->target_hom_classes_ptr));
    printf(
        "\nA* terminated with the inputs:\n"
        "\t(x_g, y_g) = (%f, %f)\n"
        "\tr = %f, s = %f\n"
        "\ta = %f, b = %f\n"
        "\tfloat tolerance = %f\n"
        "\tabsolute tolerance for homotopy classes: %f\n"
        "\tMaximum homotopy classes: %d\n"
        "\tTarget homotopy classes: %s\n"
        "\tFilled homotopy classes: %d\n"
        "\tMaximum expandible states: %d\n"
        "\tTotal states expanded: %d\n"
        "\tCurrent size of the open set (enqueued states): %d\n",
    args->params->x_g, args->params->y_g, args->params->r, args->params->s, 
    args->config->a, args->config->b, args->float_tol, args->abs_tol, args->max_hom_classes, hom_classes_str,
    filled_hom_classes, max_expandible_states, expanded_states, open_set_size(open_set) 
    );
    free(hom_classes_str);
    #endif
    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf(
        "\nTime elapsed since invoking A_star_homotopies until end of execution:\n"
        "\t%.5f seconds\n\n", 
        elapsed
    );
    #endif

    free_open_set(open_set); // redundant since set should be empty by this point 
    // nontrivial destructor, as it spares the just-initialized target homotopy list
    deallocate_grid(args, S, target_hom_classes_init, x_goal_step, y_goal_step);
    
}   // A* homotopies end



/*********** helpers ***************/
hom_vertex_t *** allocate_grid(struct A_star_homotopies_args * args) {

    hom_vertex_t *** S = malloc(args->params->ys_steps * sizeof(hom_vertex_t **));  // row pointers
    if (!S) {
        fprintf(stderr, "Failed to allocate memory for grid rows.\n");
        return NULL;
    }
    for (int ys = 0; ys < args->params->ys_steps; ys++) {
        S[ys] = (hom_vertex_t **) malloc(args->params->xs_steps * sizeof(hom_vertex_t *)); // allocate column
        if (!S[ys]) {
            fprintf(stderr, "Failed to allocate memory for grid columns at row %d.\n", ys);
            // Free previously allocated memory before exiting
            for (int j = 0; j < ys; j++) {
                free(S[j]);
            }
            free(S);
            return NULL;
        }
        for (int xs = 0; xs < args->params->xs_steps; xs++) {
            /** VERTEX ALLOCATION:
             *  1) limit each vertex in the grid to the desired maximum homotopy classes
             *  2) use g-score as comparator to select homotopy classes represented per vertex
             * NOTE: ignoring NULL returns
             */ 
            S[ys][xs] = hom_vertex_new(xs, ys, args->max_hom_classes, g_score_compare);
        }
    }
    return S;
}

void deallocate_grid(struct A_star_homotopies_args * args, hom_vertex_t *** S, bool target_hom_classes_init, int x_goal_step, int y_goal_step) {
    for (int ys = 0; ys < args->params->ys_steps; ys++) {
        for (int xs = 0; xs < args->params->xs_steps; xs++) {
            hom_vertex_t * curr_vert = S[ys][xs];
            // Prevent deletion if we have passed the reference of the goal vertex's homotopy class list
            if (!target_hom_classes_init || xs != x_goal_step || ys != y_goal_step) {
                free_hom_classes_list(curr_vert->hom_classes);
            } 
            free(curr_vert); // free vertex * within matrix
        }
        free(S[ys]); // free row pointer (vertex **)
    } 
    free(S);
}


/*********** Unit testing area *****************
 * 
 * Usage example and testing for the function A_star_homotopies
 * 
*/

#ifdef AST_UT

int main(int argc, char *argv[]) {

    /* Hard-coded constants */
    const float float_tol = 0.05;
    const double abs_tol = 0.1;
  
    /* Inputs initialized from the command line */
    char * input_json_path;
    int max_expandible_states_mult = 0;
    int max_hom_classes = 0;
    float a = 0.0f;
    float b = 0.0f;

    /* Local variables for A* */
    Params * params;
    Config * config;

    // Parsing command-line arguments
    if (argc != 6) { 
        fprintf(stderr, "Usage: %s <input_json_path> <a> <b> <max_expandible_states_mult> <max_hom_classes> \n", argv[0]);
        return 1;  
    }

    PARSE_STRING(input_json_path, argv[1], "<input_json_path>");
    PARSE_FLOAT(argv[2], a);
    PARSE_FLOAT(argv[3], b);
    PARSE_INT(argv[4], max_expandible_states_mult);
    PARSE_INT(argv[5], max_hom_classes);

    params = load_json(input_json_path);
    if (params == NULL) {
        DEBUG_ERROR("load_json / extract_parameters returned NULL, exiting...");
        exit(1);
    }

    config = malloc(sizeof(Config));
    config->a = a;
    config->b = b;
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);
    // const int max_hom_classes = params->num_obstacles + (params->num_obstacles / 2); // 50% more than the number of obstacles
    hom_classes_list_t * target_hom_classes = NULL; // initialization value

    // Log parsed parameters
    #ifdef AST_HC_DBG
    printf(
        "Running A* homotopies with inputs from command-line " 
        "a = %f; b = %f; max_expandible_states_mult = %d; max_hom_classes = %d\n", 
        a, b, max_expandible_states_mult, max_hom_classes
    ); 
    #endif

    struct A_star_homotopies_args  * astar_args = malloc(sizeof(struct A_star_homotopies_args));
/** NOTE: Ensure the inputs are properly updated as we change the struct!*/
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
    // test updating target_hom_classes with a couple iterations
    for (int i = 0; i < 4; i++) {
        A_star_homotopies(astar_args);
        // Tweak the cost function to change the order in which homotopy classes are discovered
        config->a *= powf(16, i);
    }

    /** IMPORTANT: order in which we free is crucial, since some structs depend on others! */
    free_hom_classes_list(target_hom_classes);  // in an iteration scenario we would save the backtrack
    free(astar_args);
    delete_obstacle_marker_func_params(F);
    free(config);
    free_params(params);
    free(input_json_path);


    return 0;
}

#endif




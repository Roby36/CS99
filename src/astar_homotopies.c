
#include "astar_homotopies.h"

/* Name of parameter containing path written to .json file */
const char * output_path_key_format = "a=%.2f, b=%.2f, L=(%.2f + %.2fi)";

/****************** cost / heuristic function implementations ****************+*/

/****** len ******
 * 
 * This function calculates the absolute value, relative to the map, of the distance between two arbitrary grid coordinates.
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the scaling factor s for the grid
 * Outputs:
 *      the resulting float-valued distance between (x_1,y_1) and (x_2,y_2), in map (not grid) units
 * 
 * This function will be applied on a series of adjacent points to calculate path lenths,
 * which will contribute to the path length linear component of the overall cost function (b-coefficient)
*/
static inline float len(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    return (params->s * D_ADJACENT(x_1, y_1, x_2, y_2)); // Use s as scaling factor
}

/****** g_image_riemann_sum ******
 * 
 * This function computes the Riemann sum, according to the (uncertainty) function g, 
 * between two arbitrary adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 * 
 * Inupts:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing: 
 *          - scaling factors s and r for the grid and function g respectively
 *          - g_image containing the pre-computed values for g at r-steps resolution, stored in a 2-dimensional matrix
 *          - bounds xr_steps, yr_steps of the matrix containing g_image
 * 
 * Output:
 *      The contribution of taking a path integral of g, between (x_1,y_1) and (x_2,y_2), scaled to map units.
 * 
 * This function will be applied on a series of adjacent points in the domain of g to calculate path integrals numerically,
 * which will contribute to the uncertainty accumulation linear component of the overall cost function (a-coefficient)    
 */
static inline float g_image_riemann_sum(
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

/****** edge_cost ******
 * 
 * This is the resulting cost function combining linearly uncertainty accumulation and path lenth,
 * which are computed by g_image_riemann_sum and len respectively
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the fixed environment parameters, passed as input for
 *          - g_image_riemann_sum
 *          - len
 *      Config struct containing the relative weights a and b for uncertainty accumulation and path length accordingly
 * 
 * Output:
 *      The resulting cost between the points (x_1,y_1) and (x_2,y_2) in the grid. Note that this is expressed in map units (floats)
 * 
 * This function will be passed as input for the A* search algorithm, serving as the main cost function for the graph search. 
 */
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


/****** heuristic ******
 * 
 * This is the main heuristic used for the basic (not homotopy class) version of A*.
 * It consists in aiming towards the goal as fast as possible, firstly by exploiting 
 * all the faster diagonal transitions available, and then finishing with the remaining straight ones.
 * For each step, we cancel the Riemann sum factor of the cost function, thereby guaranteeing
 * admissibility and swifter computation. 
 * 
 * Inputs:
 *      current point on the grid (x_n, y_n), and goal point (x_g, y_g)
 *      Config struct holding b coefficient of the cost function (the coefficient determining the weight for path length) 
 *      Params struct holding scaling factor s for the grid
 * 
 * Outputs:
 *      Resulting value of the heuristic between (x_n, y_n) and (x_g, y_g), 
 *          guaranteed to be lower than the lower cost path between the two points
 * 
 * This function is passed as parameter to the A* search algorithm, when this is merely looking for 
 * the lowest cost path (not the more general version keeping track of all the homotopy classes). The tightness of this
 * heuristic significantly accelerates the graph search procedure towards the goal
 */
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

/****** zero_heuristic ******
 * 
 * This is a trivial zero heuristic, used to turn A* into Dijkstra's algorithm in the case where
 * we are concerned in keeping track of all the possible homotopy classes.
 * In that case, we want to guarantee as much uniform and unbiased coverage as possible of the environment.
 * 
 * The dummy arguments make the function signature conform to the A* algorithm main function, 
 * thereby making the function reusable with different heuristic
 */
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config)
{
    return 0.0f;
}


/************ A_star_homotopies static helper functions *************/

/****** expand_backtrack *******
 * 
 * This function uses the information stored within the hom_class_t by the A* procedure
 * to carefully backtrack and allocate the path of the homotopy class. It is convenient because
 * it allows us to arbitrarily determine when a path is worth allocating and expanding at any time
 * we want to do so, by just using lightweight information stored in the homotopy class struct.
 * 
 * Inputs:
 *      Params struct for the given environment
 *      homotopy class struct pointer for which we want to expand and store the path
 * 
 * Outputs:
 *      The nested struct member hom_class->backtrack is updated with all the information on the path
 * 
 * NOTE: Caller must then call the corresponding clean-up function free_backtrack 
 *       which carefully free's all the memory malloc'd by this function
 * 
*/
void expand_backtrack(Params *params, hom_class_t * hom_class) {
    
    int edge_number     = 0;       // number of discrete edges
    float abs_len_cumul = 0.0f;    // cumulative length of path, scaled up
    float riem_cumul    = 0.0f;    // cumulative rieamnn sum for g_image buildup, scaled up
    int x_curr, y_curr;            // backtracking coordinates
    int x_next, y_next;         

    // First, find the length of the path by traversing from goal to start
    hom_class_t * curr_hom_class = hom_class;
    while (curr_hom_class != NULL) {
        edge_number++;
        curr_hom_class = curr_hom_class->parent;
    }

    // Allocate backtracking path structure from here 
    if (hom_class->backtrack == NULL) {
        hom_class->backtrack = (Backtrack_Path *) malloc (sizeof(Backtrack_Path));
    }

    Backtrack_Path * bt = hom_class->backtrack;

    // Set length for path struct
    bt->total_edges = edge_number;

    // Allocate memory for the points list
    bt->point_list = malloc(sizeof(float *) * edge_number);
    for (int i = 0; i < edge_number; i++) {
        bt->point_list[i] = malloc(sizeof(float) * 2); // Each point has two coordinates (x, y)
        if (!bt->point_list[i]) {
            fprintf(stderr, "Memory allocation failed for point_list[%d].\n", i);
            return;
        }
    }

    // Backtrack from goal to start and store the path in reverse
    curr_hom_class = hom_class;
    for (int i = edge_number - 1; i >= 0; i--) {

        // Extract coordinates from current vertex and parent if there is one
        x_curr = x_next = curr_hom_class->endpoint_vertex->x_s;
        y_curr = y_next = curr_hom_class->endpoint_vertex->y_s;
        if (curr_hom_class->parent != NULL) {
            x_next = curr_hom_class->parent->endpoint_vertex->x_s;
            y_next = curr_hom_class->parent->endpoint_vertex->y_s;
        }

        // Re-compute length and Riemann sum as they were originally computed, this time without the constants a,b
        abs_len_cumul  +=                 len(x_next, y_next, x_curr, y_curr, params);
        riem_cumul     += g_image_riemann_sum(x_next, y_next, x_curr, y_curr, params);

        // Record points in the array containing the path
        bt->point_list[i][0] = params->x_min + (x_curr * params->s); // Convert x_s back to x
        bt->point_list[i][1] = params->y_min + (y_curr * params->s); // Convert y_s back to y

        #ifdef ASTDBG
        printf("Backtracking path point[%d]: (%.2f, %.2f)\n", i, bt->point_list[i][0], bt->point_list[i][1]);
        #endif

        curr_hom_class = curr_hom_class->parent;    // iterate backwards through vertices on optimal path
    }

    // Set elements for the path
    bt->g_image_riemann_tot = riem_cumul;
    bt->absolute_length     = abs_len_cumul;

}

/****** write_path *******
 * 
 * This function writes the points of the output path corresponding to some 
 * cost coefficients a,b and homotopy class to the .json output file,
 * so that it can be parsed and rendered by graphing software (i.e. matplotlib)
 * 
 * Inputs:
 *      homotopy class object containing the path
 *      config struct containing a,b coefficients for cost function on which path was evaluated
 * 
 * Outputs:
 *      Path points written out as a matrix to ./params.json file (name hardcoded)
 * 
 * NOTE: The caller must first invoke expand_backtrack(params, hom_class)
 *       to ensure that the backtrack path is both computed and stored in hom_class->backtrack,
 *       and, preferably, invoke write_path before free_backtrack which deletes the path
 * 
 * Usage example:
 * 
 *  expand_backtrack(params, hom_class);
 *  // do something with the path
 *  write_path(hom_class, config);
 *  // do something with the path
 *  free_backtrack(hom_class);
 * 
 */
void write_path(hom_class_t * hom_class, Config * config) {

    Backtrack_Path * bt = hom_class->backtrack;
    if (bt == NULL) return;
    const int str_size = 256;
    char * Lval_string = (char*) malloc(str_size);

    // Format the entry for the path in the output .json file
    snprintf(
        Lval_string, str_size, 
        output_path_key_format, 
        config->a, config->b, creal(hom_class->Lval), cimag(hom_class->Lval)
    );
    #ifdef AST_HC_DBG
    printf("Writing path to ./params.json file with key: %s\n", Lval_string);
    #endif
    write_json("./params.json", Lval_string, bt->point_list, bt->total_edges - 1, 2);
    free(Lval_string);
}

/****** free_backtrack ******
 * 
 * The corresponding clean-up routine to expand_backtrack.
 * 
 * Inputs:
 *      homotopy class struct whose backtrack data member we want to free
 * 
 * Outputs:
 *      The function carefully deallocates all the memory taken up by
 *      hom_class->backtrack 
*/
void free_backtrack(hom_class_t * hom_class) {

    Backtrack_Path * bt = hom_class->backtrack;

    // Verify that both path struct and internal point list are accessible 
    if (bt == NULL || bt->point_list == NULL || bt->total_edges == 0) {
        DEBUG_ERROR("free_point_array: opt_path = NULL OR opt_path->point_list = NULL OR opt_path->l = 0");
        return;
    }

    // Edge iteration through array
    for (int i = 0; i < bt->total_edges; i++) {
        // Verify that we are actually free'ing
        if (bt->point_list[i] == NULL) {
            DEBUG_ERROR("free_point_array: unexpected NULL value for opt_path->point_list[i]");
        }
        // Free the point
        free(bt->point_list[i]);
    }

    // Finally, free the backtrack structure pointer itself
    free(bt); 
}


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

    // Flag current target homotopy classes as not updated by this run
    if (*(args->target_hom_classes_ptr) != NULL) {
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

    /** SUBROUTINE: Here we initialize the vertices, with empty lists of homotopy classes */
    hom_vertex_t *** S = malloc(args->params->ys_steps * sizeof(hom_vertex_t **));  // row pointers
    if (!S) {
        fprintf(stderr, "Failed to allocate memory for grid rows.\n");
        return;
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
            return;
        }
        for (int xs = 0; xs < args->params->xs_steps; xs++) {
            /** VERTEX_INITIALIZATION: **/
            hom_vertex_t * new_vertex = malloc(sizeof(hom_vertex_t));
            new_vertex->x_s = xs;
            new_vertex->y_s = ys;
            new_vertex->hom_classes = hom_classes_list_new(
                args->max_hom_classes,  // limit each vertex in the grid to the desired maximum homotopy classes
                g_score_compare         // use g-score as comparator to select homotopy classes represented per vertex
            ); // intialize empty list of Lvalues for each vertex
            S[ys][xs] = new_vertex; // Save malloc'd vertex to grid matrix
        }
    }

    // initialize start vertex AND start homotopy class 
    hom_vertex_t * start_vertex = S[y_start_step][x_start_step];
    hom_class_t * start_hom_class = (hom_class_t *) malloc(sizeof(hom_class_t));
    start_hom_class->Lval = CMPLX(0.0, 0.0);
    start_hom_class->next = NULL;   
    start_hom_class->updated = true; // new homotopy class contructed in this run of A*
    start_hom_class->parent = NULL;
    start_hom_class->g_score = 0.0f;
    start_hom_class->f_score = args->h(
        x_start_step, y_start_step, x_goal_step, y_goal_step, args->params, args->config
    );
    start_hom_class->is_evaluated = false;
    start_hom_class->endpoint_vertex = start_vertex;
    start_hom_class->minheap_node = NULL;
    start_hom_class->backtrack = NULL;
    insert_hom_class(start_vertex->hom_classes, start_hom_class, args->abs_tol); // assume the start homotopy class qualifies 

    // Initialize list of target homotopy classes, if necessary, to the pointer to the list of the goal vertex 
    if (*(args->target_hom_classes_ptr) == NULL) {    
        *(args->target_hom_classes_ptr) = S[y_goal_step][x_goal_step]->hom_classes;
    }

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
            #ifdef ASTDBG
            printf(
                "Goal reached at (%d, %d) from L-value (%.2f + %.2fi) with total path cost (g_score): %f\n", 
                x_c, y_c, creal(hom_class->Lval), cimag(hom_class->Lval), hom_class->g_score
            );
            #endif

            expand_backtrack(args->params, curr_hom_class);

            /** NOTE: Since hom classes are dequeued only once, if dequeued here, guaranteed to be best possible, hence we update it in place */
            hom_class_t * target_hom_class = hom_class_get(*(args->target_hom_classes_ptr), curr_hom_class->Lval, args->abs_tol);
            if (target_hom_class != NULL) {
                /** NOTE: ensure that we are not writing on this homotopy class itself **/
                if (target_hom_class != curr_hom_class) {
                    free_backtrack(target_hom_class);               
                    hom_class_t * next = target_hom_class->next;    /** CRITICAL: preserve pointer to next element, else you break *target_hom_classes_ptr list! */
                    *(target_hom_class) = *(curr_hom_class);        // copy in new homotopy class at the same adress
                    target_hom_class->next = next;                  // restore next pointer to preserve list structure                      
                }
                filled_hom_classes++;   // if we dequeued a class in the targets, then we have filled it by here
            }

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
        
            write_path(curr_hom_class, args->config); // write path to ./params.json
            // free_backtrack(curr_hom_class); // leave path allocated here
        }
        // End of goal state subroutine 

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
            // If not, initialize it
            if (nbr_hom_class == NULL) {
                discovered_nbr_class = true;
                nbr_hom_class = (hom_class_t *) malloc(sizeof(hom_class_t));
                nbr_hom_class->Lval = neighb_Lval;
                nbr_hom_class->next = NULL; 
                nbr_hom_class->updated = true; // constructed in this run of A*
                nbr_hom_class->parent = NULL;
                nbr_hom_class->g_score = INFINITY;
                nbr_hom_class->f_score = INFINITY;
                nbr_hom_class->is_evaluated = false;
                nbr_hom_class->endpoint_vertex = neighb_vertex;
                nbr_hom_class->minheap_node = NULL;
                nbr_hom_class->backtrack = NULL; 
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
        
            /** CRITICAL: If the homotopy class is already in the min heap, dedrease key, otherwise add it */
            if (nbr_hom_class->minheap_node) {
                decrease_key(open_set, nbr_hom_class->minheap_node, nbr_hom_class->f_score);
            } else {
                insert_sorted(open_set, nbr_hom_class);
            }
        }
        }
    }

    // Add general print statement and timestamp
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

    // Clean up 
    #ifdef ASTDBG
    printf("Cleaning up resources...\n");
    #endif

    free_open_set(open_set); 

    /** IMPORTANT: Grid deallocation, saving the hom_classes_list for the goal vertex as this is the collateral output, used for iterative runs */ 
    for (int ys = 0; ys < args->params->ys_steps; ys++) {
        for (int xs = 0; xs < args->params->xs_steps; xs++) {
            hom_vertex_t * curr_vert = S[ys][xs];
            // This free's all the dynamically allocated homotopy classes, except for the goal vertex
            if (xs != x_goal_step || ys != y_goal_step) {
                free_hom_classes_list(curr_vert->hom_classes);
            } 
            free(curr_vert); // free vertex * within matrix
        }
        free(S[ys]); // free row pointer (vertex **)
    } 
    free(S);
    
    // A* homotopies end
}


/*********** Unit testing area *****************
 * Demonstrating some use-cases and initializations required 
 * to run the main algorithm
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

    input_json_path = (char *) malloc(strlen(argv[1]) + 1);
    if (input_json_path == NULL) {
        fprintf(stderr, "Memory for <input_json_path> string allocation failed\n");
        return 1;
    }
    strcpy(input_json_path, argv[1]);

    PARSE_FLOAT(argv[2], a, 2);
    PARSE_FLOAT(argv[3], b, 3);
    PARSE_INT(argv[4], max_expandible_states_mult, 4);
    PARSE_INT(argv[5], max_hom_classes, 5);

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
    A_star_homotopies(astar_args);

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



